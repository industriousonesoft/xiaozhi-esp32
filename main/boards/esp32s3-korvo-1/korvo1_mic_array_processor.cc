#include "korvo1_mic_array_processor.h"

#include <algorithm>
#include <cmath>

#ifdef ESP_PLATFORM
#include <esp_log.h>

extern "C" void* mase_create(int fs, int frame_size, int array_type, float mic_distance,
                             int operating_mode, int filter_strength) __attribute__((weak));
extern "C" void mase_process(void* st, int16_t* in, int16_t* dsp_out) __attribute__((weak));
extern "C" void mase_destory(void* st) __attribute__((weak));
#endif

namespace {
constexpr const char* TAG = "Korvo1MicArray";

#ifdef ESP_PLATFORM
#define KORVO1_MIC_ARRAY_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define KORVO1_MIC_ARRAY_LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#else
#define KORVO1_MIC_ARRAY_LOGI(...) do {} while (0)
#define KORVO1_MIC_ARRAY_LOGW(...) do {} while (0)
#endif

// ES7210 原始 I2S 读出按 4 路 int16 lane 处理。根据 esp-skainet Korvo-1 BSP，lane0 是回声参考；
// Korvo-1 硬件有三颗模拟麦，v2 默认把 lane1/lane2/lane3 都作为麦阵输入。
constexpr int kRawChannels = 4;
constexpr int kRawRefIndex = 0;
constexpr int kRawMic0Index = 1;
constexpr int kRawMic1Index = 2;
constexpr int kRawMic2Index = 3;

// 麦阵输出给后级 AFE 的格式固定为 M/R：一路增强后的主声源 + 一路播放参考。
constexpr int kAfeChannels = 2;
constexpr int kMicArrayChannels = 3;

// ESP-SR MASE 文档约束：16 kHz、16 ms，也就是 256 samples/frame。
constexpr int kMaseFrameSamples = 256;
constexpr int kMaseSampleRate = 16000;
constexpr int kMaseFrameMs = 16;
constexpr int kThreeMicCircle = 1;
constexpr int kNormalEnhancementMode = 0;
constexpr float kMicDistanceMm = 65.0f;
constexpr int kMaseFilterStrength = 2;

// fallback 算法的经验门限。这里判断的是三路麦里最强一路的 10ms/16ms RMS。
// 太低会被底噪触发，太高会漏掉远距离/小音量说话。
constexpr float kMinRmsForVoice = 280.0f;
constexpr float kFullScaleRms = 3500.0f;
constexpr float kMinConfidence = 0.18f;
constexpr uint32_t kDebugLogIntervalMs = 500;
constexpr float kPi = 3.14159265358979323846f;

// Korvo-1 三麦在 fallback 中按圆阵近似建模。真实零度和顺逆时针可通过 LED 映射宏校准；
// 若实测通道物理顺序不同，优先调整 raw mic index，而不是改通用算法。
constexpr float kMicAnglesDeg[kMicArrayChannels] = {0.0f, 120.0f, 240.0f};

float Clamp(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

float DegToRad(float angle_deg) {
    return angle_deg * kPi / 180.0f;
}

float RadToDeg(float angle_rad) {
    return angle_rad * 180.0f / kPi;
}

int16_t ClampToInt16(float value) {
    value = Clamp(value, -32768.0f, 32767.0f);
    return static_cast<int16_t>(std::lround(value));
}
} // namespace

Korvo1MicArrayProcessor::~Korvo1MicArrayProcessor() {
#ifdef ESP_PLATFORM
    if (mase_handle_ != nullptr && mase_destory != nullptr) {
        mase_destory(mase_handle_);
    }
#endif
}

Korvo1MicArrayResult Korvo1MicArrayProcessor::ProcessRaw(const int16_t* raw_data, size_t raw_samples,
                                                         std::vector<int16_t>& output_mr,
                                                         uint32_t timestamp_ms) {
    Korvo1MicArrayResult result;
    result.timestamp_ms = timestamp_ms;
    output_mr.clear();

    if (raw_data == nullptr || raw_samples < kRawChannels || raw_samples % kRawChannels != 0) {
        if (ShouldLogDebug(timestamp_ms)) {
            KORVO1_MIC_ARRAY_LOGI("inactive: invalid raw frame data=%p samples=%u",
                                  raw_data, static_cast<unsigned>(raw_samples));
        }
        return result;
    }

    const size_t frames = raw_samples / kRawChannels;
    std::vector<int16_t> mic_data(frames * kMicArrayChannels);
    std::vector<int16_t> ref_data(frames);
    for (size_t i = 0; i < frames; ++i) {
        ref_data[i] = raw_data[kRawChannels * i + kRawRefIndex];
        mic_data[kMicArrayChannels * i + 0] = raw_data[kRawChannels * i + kRawMic0Index];
        mic_data[kMicArrayChannels * i + 1] = raw_data[kRawChannels * i + kRawMic1Index];
        mic_data[kMicArrayChannels * i + 2] = raw_data[kRawChannels * i + kRawMic2Index];
    }

    std::vector<int16_t> enhanced_mic;
    bool used_mase = TryMaseProcess(mic_data.data(), frames, enhanced_mic);
    if (used_mase) {
        std::vector<int16_t> direction_probe;
        result = ProcessThreeMicFallback(mic_data.data(), frames, direction_probe, timestamp_ms);
        if (ShouldLogDebug(timestamp_ms)) {
            KORVO1_MIC_ARRAY_LOGI("beamformer: MASE enhanced frame frames=%u angle=%.1f conf=%.2f active=%d",
                                  static_cast<unsigned>(frames), result.angle_deg, result.confidence, result.active);
        }
    } else {
        result = ProcessThreeMicFallback(mic_data.data(), frames, enhanced_mic, timestamp_ms);
    }

    output_mr.resize(frames * kAfeChannels);
    for (size_t i = 0; i < frames; ++i) {
        output_mr[kAfeChannels * i + 0] = enhanced_mic[i];
        output_mr[kAfeChannels * i + 1] = ref_data[i];
    }

    return result;
}

Korvo1MicArrayResult Korvo1MicArrayProcessor::Process(const int16_t* data, size_t samples, uint32_t timestamp_ms) {
    std::vector<int16_t> output_mr;
    return ProcessRaw(data, samples, output_mr, timestamp_ms);
}

bool Korvo1MicArrayProcessor::TryMaseProcess(const int16_t* mic_data, size_t frames,
                                             std::vector<int16_t>& output_mic) {
#ifdef ESP_PLATFORM
    if (mase_create == nullptr || mase_process == nullptr) {
        return false;
    }
    if (frames != kMaseFrameSamples) {
        return false;
    }
    if (mase_handle_ == nullptr) {
        mase_handle_ = mase_create(kMaseSampleRate, kMaseFrameMs, kThreeMicCircle, kMicDistanceMm,
                                   kNormalEnhancementMode, kMaseFilterStrength);
        if (mase_handle_ == nullptr) {
            KORVO1_MIC_ARRAY_LOGW("mase_create failed, fallback beamformer will be used");
            return false;
        }
    }

    output_mic.resize(frames);
    mase_process(mase_handle_, const_cast<int16_t*>(mic_data), output_mic.data());
    return true;
#else
    (void)mic_data;
    (void)frames;
    (void)output_mic;
    return false;
#endif
}

Korvo1MicArrayResult Korvo1MicArrayProcessor::ProcessThreeMicFallback(const int16_t* mic_data, size_t frames,
                                                                      std::vector<int16_t>& output_mic,
                                                                      uint32_t timestamp_ms) {
    // fallback 是没有可用 MASE 库时的轻量麦阵路径。
    // 目标不是做严格的物理级 beamforming，而是先完成两个工程目标：
    //   1. 从 3 路 Mic 中估计一个“当前主声源方向”，给 12 颗 LED 使用；
    //   2. 根据这个方向做一个方向性加权混合，输出 1 路增强 Mic 给 AFE。
    //
    // 输入 mic_data 的格式固定为 3 路交织：
    //   mic_data[3*i + 0] = Mic0
    //   mic_data[3*i + 1] = Mic1
    //   mic_data[3*i + 2] = Mic2
    //
    // 输出 output_mic 是单路 mono PCM，后续会和 REF 拼成 M/R 送入 AFE。
    // 注意：REF 不在这个函数里处理，因为 REF 不是环境声源，不能参与方向估计。
    Korvo1MicArrayResult result;
    result.timestamp_ms = timestamp_ms;
    output_mic.resize(frames);

    if (mic_data == nullptr || frames == 0) {
        return result;
    }

    double energy[kMicArrayChannels] = {};
    for (size_t i = 0; i < frames; ++i) {
        for (int ch = 0; ch < kMicArrayChannels; ++ch) {
            // 累加平方能量。平方有两个作用：
            //   1. 把正负波形都变成正能量，避免正负样本互相抵消；
            //   2. 对大幅度声音更敏感，适合做短时响度判断。
            const float sample = static_cast<float>(mic_data[kMicArrayChannels * i + ch]);
            energy[ch] += sample * sample;
        }
    }

    float rms[kMicArrayChannels] = {};
    float max_rms = 0.0f;
    float sum_rms = 0.0f;
    for (int ch = 0; ch < kMicArrayChannels; ++ch) {
        // RMS = sqrt(mean(sample^2))，可以理解成这一帧的平均响度。
        // 相比单点峰值，RMS 不容易被偶发尖峰带偏，更适合判断“这一路麦有没有稳定声音”。
        rms[ch] = std::sqrt(energy[ch] / frames);
        max_rms = std::max(max_rms, rms[ch]);
        sum_rms += rms[ch];
    }

    if (max_rms < kMinRmsForVoice || sum_rms <= 1.0f) {
        // 三路里最强一路都没有超过门限，认为当前没有可靠主声源。
        // 这里仍然输出 Mic0，而不是输出全 0：
        //   - 可以保证后级 AFE/调试链路持续收到合法音频；
        //   - 静音/低声场景下不会因为 fallback 而切断输入。
        for (size_t i = 0; i < frames; ++i) {
            output_mic[i] = mic_data[kMicArrayChannels * i + 0];
        }
        // 声音消失时不要立刻清空历史方向，先衰减置信度。
        // 这样 LED 在一句话末尾会自然淡出，不会因为一两帧低能量马上跳回默认状态。
        last_confidence_ *= 0.75f;
        if (last_confidence_ < kMinConfidence) {
            has_estimate_ = false;
        }
        if (ShouldLogDebug(timestamp_ms)) {
            KORVO1_MIC_ARRAY_LOGI("inactive: three-mic low energy frames=%u rms=[%.1f %.1f %.1f] threshold=%.1f",
                                  static_cast<unsigned>(frames), rms[0], rms[1], rms[2], kMinRmsForVoice);
        }
        return result;
    }

    float x = 0.0f;
    float y = 0.0f;
    for (int ch = 0; ch < kMicArrayChannels; ++ch) {
        // 把每颗麦的 RMS 当作“这个方向上的投票权重”，投到一个二维方向向量上。
        // kMicAnglesDeg 是三颗麦在算法里的近似方位：
        //   Mic0 = 0 度
        //   Mic1 = 120 度
        //   Mic2 = 240 度
        //
        // 如果某一路 RMS 更大，最终合成向量会被拉向那颗麦的方向。
        // 这是一种很粗的方向估计：它使用的是能量差，不是 TDOA/GCC-PHAT 的到达时间差。
        // 优点是计算极轻；缺点是容易受麦克风增益差、外壳遮挡、混响和多人同时说话影响。
        const float angle = DegToRad(kMicAnglesDeg[ch]);
        x += rms[ch] * std::cos(angle);
        y += rms[ch] * std::sin(angle);
    }

    // vector_strength 表示三路能量的方向一致性：
    //   接近 0：三路能量差不多，方向信息很弱，可能是正前方、远场混响或均匀噪声；
    //   接近 1：某一路或某个方向明显更强，方向信息更可靠。
    //
    // 当三路 RMS 几乎相等时，理论上 x/y 会接近 0，atan2 的角度会很不稳定。
    // 因此 vector_strength < 0.05 时强制认为声源在 0 度，也就是 LED0 正前方。
    const float vector_strength = std::sqrt(x * x + y * y) / sum_rms;
    float raw_angle = vector_strength < 0.05f ? 0.0f : NormalizeAngle(RadToDeg(std::atan2(y, x)));

    // confidence 是给 LED 和抖动抑制使用的“可信度”，不是概率。
    // 第一部分来自声音强度：max_rms 越高，越像真实近端说话人；
    // kMinRmsForVoice 以下为 0，kFullScaleRms 附近趋近 1。
    float confidence = Clamp((max_rms - kMinRmsForVoice) / (kFullScaleRms - kMinRmsForVoice), 0.0f, 1.0f);

    // 第二部分来自方向一致性。方向向量越明显，越相信角度判断。
    // 下限 0.30 是为了正前方说话不被误判为 inactive：
    // 正前方可能导致三路能量接近，但它仍然是有效语音，只是方向特征不强。
    const float direction_strength = Clamp(vector_strength * 3.0f, 0.30f, 1.0f);
    confidence *= direction_strength;

    if (has_estimate_) {
        // 稳定性抑制：如果本帧角度和上一帧差很多，降低置信度。
        // 这能减少多人说话、敲击、反射声导致 LED 在两个方向之间高速跳变。
        // 180 度差异时仍保留 0.35 的下限，避免真正换人说话时完全锁死旧方向。
        const float distance = AngleDistance(raw_angle, last_angle_deg_);
        confidence *= Clamp(1.0f - distance / 180.0f, 0.35f, 1.0f);
    }

    const float raw_rad = DegToRad(raw_angle);
    // 对角度做平滑时不能直接平均角度值，因为角度有 0/360 环绕问题：
    // 例如 359 度和 1 度的平均应该接近 0 度，而不是 180 度。
    // 所以这里把角度转成单位圆上的 x/y 向量做低通滤波，再 atan2 转回角度。
    //
    // alpha 是新观测的权重：
    //   第一次估计时 alpha=1，立即采用当前方向；
    //   后续根据 confidence 取 0.25~0.70，置信度高时跟随更快，置信度低时更稳。
    const float alpha = has_estimate_ ? Clamp(0.25f + confidence * 0.45f, 0.25f, 0.70f) : 1.0f;
    smoothed_x_ = smoothed_x_ * (1.0f - alpha) + std::cos(raw_rad) * alpha;
    smoothed_y_ = smoothed_y_ * (1.0f - alpha) + std::sin(raw_rad) * alpha;

    last_angle_deg_ = NormalizeAngle(RadToDeg(std::atan2(smoothed_y_, smoothed_x_)));
    last_confidence_ = confidence;
    has_estimate_ = confidence >= kMinConfidence;

    // Delay-and-sum 的完整版本需要按阵列几何和声速做亚采样延时。fallback 第一版先做方向性加权混合：
    // 主方向对应的麦权重大，背向麦权重小；这样可以在不引入帧延迟的情况下先把 M/R 主链路打通。
    float weights[kMicArrayChannels] = {};
    float weight_sum = 0.0f;
    for (int ch = 0; ch < kMicArrayChannels; ++ch) {
        // 计算当前主方向和每颗麦“算法方位”的夹角。
        // 夹角越小，说明这颗麦更靠近主声源方向，权重越高；
        // 夹角越大，尤其在背向时，权重接近最低值。
        const float diff = DegToRad(AngleDistance(last_angle_deg_, kMicAnglesDeg[ch]));

        // 权重范围约为 0.05~1.0：
        //   0.05 保留少量背向麦声音，避免单路麦异常/遮挡时输出完全崩掉；
        //   0.95 * cos(diff) 让正向麦明显占优，形成一个很轻量的“方向性拾音”效果。
        //
        // 这还不是严格 delay-and-sum beamforming，因为没有根据声源方向对每路麦做时间延迟补偿；
        // 它更像“方向性加权混音”。好处是无额外帧延迟、CPU 很低、容易实机调参。
        weights[ch] = 0.05f + 0.95f * std::max(0.0f, std::cos(diff));
        weight_sum += weights[ch];
    }
    if (weight_sum < 0.001f) {
        // 理论上不会发生。保底避免除 0，出现时等价于不归一化放大。
        weight_sum = 1.0f;
    }

    for (size_t i = 0; i < frames; ++i) {
        float mixed = 0.0f;
        for (int ch = 0; ch < kMicArrayChannels; ++ch) {
            mixed += mic_data[kMicArrayChannels * i + ch] * weights[ch];
        }
        // 除以权重和做归一化，避免多个麦叠加后整体音量过大。
        // ClampToInt16 防止混合结果超过 int16 PCM 范围导致溢出。
        output_mic[i] = ClampToInt16(mixed / weight_sum);
    }

    // active 表示当前方向估计是否足够可信；即使 active=false，output_mic 也仍然是可用的音频。
    // angle_deg 使用 LED0=正前方的相对角度，后续由 board 层映射到 12 颗 WS2812C。
    // confidence 用于 LED 亮度/扇区强度，也可以用来调试门限。
    result.active = has_estimate_;
    result.angle_deg = last_angle_deg_;
    result.confidence = confidence;

    if (ShouldLogDebug(timestamp_ms)) {
        KORVO1_MIC_ARRAY_LOGI(
            "fallback: frames=%u rms=[%.1f %.1f %.1f] raw_angle=%.1f angle=%.1f conf=%.2f weights=[%.2f %.2f %.2f] active=%d",
            static_cast<unsigned>(frames), rms[0], rms[1], rms[2], raw_angle, result.angle_deg, result.confidence,
            weights[0], weights[1], weights[2], result.active);
    }

    return result;
}

void Korvo1MicArrayProcessor::Reset() {
    has_estimate_ = false;
    smoothed_x_ = 1.0f;
    smoothed_y_ = 0.0f;
    last_angle_deg_ = 0.0f;
    last_confidence_ = 0.0f;
}

bool Korvo1MicArrayProcessor::ShouldLogDebug(uint32_t timestamp_ms) {
    if (timestamp_ms == 0 || last_debug_log_ms_ == 0 ||
        timestamp_ms - last_debug_log_ms_ >= kDebugLogIntervalMs) {
        last_debug_log_ms_ = timestamp_ms;
        return true;
    }
    return false;
}

float Korvo1MicArrayProcessor::NormalizeAngle(float angle_deg) {
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

float Korvo1MicArrayProcessor::AngleDistance(float a, float b) {
    float distance = std::abs(NormalizeAngle(a) - NormalizeAngle(b));
    return std::min(distance, 360.0f - distance);
}
