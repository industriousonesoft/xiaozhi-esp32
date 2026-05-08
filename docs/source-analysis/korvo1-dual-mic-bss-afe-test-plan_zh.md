# ESP32-S3-Korvo-1 双麦 BSS AFE 量化测试方案

本文给出 ESP32-S3-Korvo-1 使用双麦克风阵列配合 ESP-SR AFE BSS 算法的测试与改造方案。目标是使用三颗板载 MIC 中的两颗组成双麦阵列，通过 AFE 的 BSS/NS/VAD 处理降低次麦方向声音对主声源的影响，并将 AFE 处理后的 PCM 通过板载扬声器播放。原有 AFE 后 PCM 编码为 Opus 并上传服务器的逻辑通过宏隔离保留。

## 1. 当前音频链路背景

Korvo-1 当前板级音频链路如下：

- 播放：I2S0 -> ES8311 -> PA -> Speaker
- 录音：I2S1 <- ES7210 <- 3 Mic + Reference

当前源码中 `Korvo1AudioCodec::Read()` 从 ES7210 读取 4 路 raw lane：

```text
raw[4*i + 0] = Reference
raw[4*i + 1] = Mic0
raw[4*i + 2] = Mic1
raw[4*i + 3] = Mic2
```

随后 `Korvo1MicArrayProcessor::ProcessRaw()` 将三路麦克风先做板级麦阵处理，并向公共音频链路输出两路：

```text
M/R = 增强后的主声源 Mic + Reference
```

`AfeAudioProcessor` 再按 `M/R` 输入格式运行 AFE，输出 16 kHz、mono、16-bit PCM。当前默认输出路径是：

```text
AFE PCM -> audio_encode_queue_ -> Opus encoder -> audio_send_queue_ -> Protocol::SendAudio() -> Server
```

## 2. 目标链路

本方案使用 `Mic0` 作为主麦，`Mic1` 作为次麦，不使用 `Mic2` 参与本次双麦 BSS 测试。输入给 AFE 的格式调整为：

```text
M/M/R = Mic0 + Mic1 + Reference
```

目标链路如下：

```text
ES7210 raw 4ch
  -> Korvo1AudioCodec::Read()
  -> 选择 Mic0 + Mic1 + Reference
  -> AFE input_format = "MMR"
  -> AFE BSS/NS/VAD/AEC
  -> AFE processed mono PCM
  -> audio_playback_queue_
  -> AudioOutputTask()
  -> Korvo1AudioCodec::Write()
  -> ES8311 / Speaker
```

原服务器上传链路保留为宏控制路径：

```text
AFE processed mono PCM
  -> audio_encode_queue_
  -> Opus encoder
  -> audio_send_queue_
  -> Protocol::SendAudio()
```

## 3. 源码改造方案

### 3.1 Korvo-1 输入通道选择

在 `main/boards/esp32s3-korvo-1/config.h` 中保留 `AUDIO_INPUT_RAW_CHANNELS = 4`，新增或调整 Korvo-1 专用输入配置：

```c
#define AUDIO_INPUT_CHANNELS 3
#define AUDIO_INPUT_REFERENCE true
```

在 `Korvo1AudioCodec::Read()` 中，当启用双麦 BSS 宏时，不再调用三麦 `Korvo1MicArrayProcessor::ProcessRaw()`，而是直接从 raw lane 重排为 `Mic0/Mic1/Reference`：

```text
dest[3*i + 0] = raw[4*i + 1]  // Mic0, 主麦
dest[3*i + 1] = raw[4*i + 2]  // Mic1, 次麦
dest[3*i + 2] = raw[4*i + 0]  // Reference
```

`Mic2` 暂不参与 BSS 测试，后续如需比较不同麦克风组合，可通过板级宏切换 raw lane 映射。

### 3.2 AFE BSS 配置

`AfeAudioProcessor::Initialize()` 当前根据 `codec_->input_channels()` 与 `codec_->input_reference()` 自动生成 `input_format`。当 Korvo-1 输出 3 路且包含 reference 时，自动得到：

```text
input_format = "MMR"
```

基于 ESP-SR AFE 文档，BSS 支持双通道麦克风处理，AFE 输入数据必须是通道交错的 16-bit、16 kHz PCM。因此本方案保持上层 `ReadAudioData(data, 16000, samples)` 行为不变，只调整 Korvo-1 codec 输出的通道内容。

当前仓库使用的 ESP-SR 组件中，BSS 对应 `afe_config_t::se_init` 字段，注释为 SE(Speech Enhancement, microphone array processing)。实现中在 Korvo-1 双麦宏打开时显式设置：

```text
afe_config->se_init = true;
ESP_LOGI(TAG, "Korvo-1 dual-mic SE(BSS) enabled");
```

NS/VAD 保持现状。AEC 继续受 `CONFIG_USE_DEVICE_AEC` 控制，Reference 通道仍位于最后一通道。

### 3.3 AFE 后 PCM 本地播放

在 `AudioService::Initialize()` 中，当前 AFE 输出回调为：

```cpp
audio_processor_->OnOutput([this](std::vector<int16_t>&& data) {
    PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue, std::move(data));
});
```

改为宏控制分流：

```cpp
audio_processor_->OnOutput([this](std::vector<int16_t>&& data) {
#if CONFIG_KORVO1_AFE_PCM_LOCAL_PLAYBACK
    PushTaskToPlaybackQueue(std::move(data), 0);
#else
    PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue, std::move(data));
#endif
});
```

新增 `PushTaskToPlaybackQueue()` 或复用现有播放队列入队逻辑，将 AFE 输出的 mono PCM 直接交给 `AudioOutputTask()`。Korvo-1 当前输出采样率为 16 kHz，AFE 输出也是 16 kHz，因此本地播放路径不需要重采样。

### 3.4 原上传逻辑宏隔离

新增 Korvo-1 专用 Kconfig：

```text
CONFIG_KORVO1_DUAL_MIC_BSS
CONFIG_KORVO1_AFE_PCM_LOCAL_PLAYBACK
```

推荐默认值：

- `CONFIG_KORVO1_DUAL_MIC_BSS=y`
- `CONFIG_KORVO1_AFE_PCM_LOCAL_PLAYBACK=n`

这样默认仍走服务器上传链路，只有测试固件显式打开本地播放时才禁用上传分支。

宏行为定义：

| 宏配置 | 行为 |
| --- | --- |
| `KORVO1_DUAL_MIC_BSS=n` | 保持现有三麦板级处理后输出 `M/R` |
| `KORVO1_DUAL_MIC_BSS=y` | 输出 `Mic0/Mic1/Reference`，AFE 使用 `MMR` |
| `KORVO1_AFE_PCM_LOCAL_PLAYBACK=n` | AFE PCM 编码 Opus 并上传服务器 |
| `KORVO1_AFE_PCM_LOCAL_PLAYBACK=y` | AFE PCM 直接进入扬声器播放队列 |

## 4. 量化测试方案

### 4.1 测试目的

量化确认 AFE BSS 后的 PCM 是否有效降低次麦方向声音，同时保证主麦方向语音不过度衰减、不明显削波、不引入不可接受延迟。

### 4.2 测试设备

- ESP32-S3-Korvo-1，烧录启用双麦 BSS 的测试固件
- 两个可控声源，分别放置在 Mic0 主方向和 Mic1 次方向
- 一台 PC，用于播放测试音频、接收 UDP debug PCM、运行离线分析脚本
- 可选外部录音设备，用于记录扬声器实际播放输出

### 4.3 数据采集

每个测试场景至少保存以下 WAV：

- 原始 Mic0 PCM
- 原始 Mic1 PCM
- AFE 后 mono PCM
- 扬声器外放回录 PCM，可选

可复用 `scripts/audio_debug_server.py` 保存 PCM 为 WAV。实现中保留现有 raw input debug 行为；当 `CONFIG_KORVO1_AFE_PCM_LOCAL_PLAYBACK=y` 且 `CONFIG_USE_AUDIO_DEBUGGER=y` 时，UDP debug 改为发送 AFE 后 mono PCM，避免 raw input 与 AFE output 混入同一个 WAV。

### 4.4 测试场景

1. 单主源：只在 Mic0 主方向播放语音。
2. 单次源：只在 Mic1 次方向播放语音。
3. 双源同时：Mic0 播放主语音，Mic1 播放干扰语音。
4. 稳态噪声：加入风扇、空调或粉噪声，验证 NS+BSS 输出。
5. 长时间稳定性：连续运行 10 分钟，确认无播放队列堆积、无明显爆音、无 watchdog。

### 4.5 指标定义

#### 次源抑制量

在只播放次源的片段中计算：

```text
Suppression_dB = 20 * log10(RMS(raw_mic1) / RMS(afe_output))
```

首轮目标：

```text
Suppression_dB >= 6 dB
```

#### 主声源保持量

在只播放主源的片段中计算：

```text
Main_Retention_dB = 20 * log10(RMS(afe_output) / RMS(raw_mic0))
```

首轮目标：

```text
Main_Retention_dB >= -3 dB
```

#### SIR/SI-SDR 改善

在双源同时播放场景中，以干净主源作为参考，计算原始 Mic0 与 AFE output 的 SI-SDR 或 SIR：

```text
Improvement_dB = Metric(afe_output, clean_main) - Metric(raw_mic0, clean_main)
```

首轮目标：

```text
Improvement_dB >= 3 dB
```

#### 削波率

统计绝对值接近 int16 满量程的采样点比例：

```text
Clipping_Rate = count(abs(sample) >= 32760) / total_samples
```

首轮目标：

```text
Clipping_Rate < 0.1%
```

#### 延迟

通过 raw Mic0 与 AFE output 的互相关估计处理延迟：

```text
Delay_ms = lag_samples / 16000 * 1000
```

本地播放模式额外测量 AFE output 到外放回录的链路延迟。首轮不设硬性阈值，记录均值、最大值和抖动范围，用于后续调参。

## 5. 离线分析脚本建议

新增离线分析脚本：

```text
scripts/acoustic_check/analyze_afe_bss.py
```

输入：

```bash
python scripts/acoustic_check/analyze_afe_bss.py \
  --raw-main raw_mic0.wav \
  --raw-secondary raw_mic1.wav \
  --afe afe_output.wav \
  --clean-main clean_main.wav \
  --out report.md
```

输出：

- RMS 表
- 次源抑制量
- 主声源保持量
- SI-SDR/SIR 改善
- 削波率
- 延迟估计
- Markdown 报告和 CSV 数据

脚本使用 Python 标准库实现，可通过以下命令做自检：

```bash
python scripts/acoustic_check/analyze_afe_bss.py --self-test
```

## 6. 验收标准

首轮验收以“可比较、可复现、可回退”为准：

- `CONFIG_KORVO1_AFE_PCM_LOCAL_PLAYBACK=n` 时，原 Opus 上传链路仍可编译并继续工作。
- `CONFIG_KORVO1_AFE_PCM_LOCAL_PLAYBACK=y` 时，AFE 后 PCM 能通过扬声器播放，且不向服务器发送 AFE 音频。
- `CONFIG_KORVO1_DUAL_MIC_BSS=y` 时，AFE 输入格式确认为 `MMR`。
- 单次源场景下，AFE 输出相对原始 Mic1 至少降低 6 dB。
- 单主源场景下，AFE 输出相对原始 Mic0 不低于 -3 dB，且削波率低于 0.1%。
- 双源同时场景下，SI-SDR 或 SIR 相对原始 Mic0 至少提升 3 dB。
- 10 分钟稳定性测试中无播放队列持续增长、无 watchdog、无明显 underrun 或爆音。

## 7. 风险与注意事项

- BSS 对双麦物理距离、方向、增益一致性敏感。若实测抑制量不足，应先校准 Mic0/Mic1 的 raw lane、增益和物理方向，再调 AFE 参数。
- Reference 通道必须保持干净且通道顺序正确，否则 AEC 会影响 BSS/NS 输出质量。
- 本地扬声器播放 AFE 后 PCM 会形成声学闭环，测试时应控制音量，必要时关闭 AEC 或固定播放增益做对比。
- 当前 ESP-SR 版本使用 `se_init` 控制 SE(BSS)，不要硬编码不存在的 BSS 专用字段。
- 本地播放模式只用于量化测试，不建议作为默认产品模式。

## 8. 参考资料

- `docs/source-analysis/korvo1-audio-chain_zh.md`
- `main/boards/esp32s3-korvo-1/korvo1_audio_codec.cc`
- `main/audio/processors/afe_audio_processor.cc`
- `main/audio/audio_service.cc`
- `scripts/acoustic_check/analyze_afe_bss.py`
- ESP-SR AFE 文档：https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/audio_front_end/README.html
- ESP-SR Component Registry：https://components.espressif.com/components/espressif/esp-sr
