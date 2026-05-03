#include "korvo1_audio_codec.h"
#include "config.h"

#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include <esp_log.h>

#include <vector>

#define TAG "Korvo1AudioCodec"

Korvo1AudioCodec::Korvo1AudioCodec(void* i2c_master_handle, int input_sample_rate, int output_sample_rate,
    gpio_num_t output_mclk, gpio_num_t output_bclk, gpio_num_t output_ws, gpio_num_t output_dout,
    gpio_num_t input_mclk, gpio_num_t input_bclk, gpio_num_t input_ws, gpio_num_t input_din,
    gpio_num_t pa_pin, uint8_t es8311_addr, uint8_t es7210_addr, int input_channels) {
    duplex_ = true;

    // Korvo-1 的 ES7210 输入里包含扬声器回采/参考通道，AFE 需要知道这一点来启用回声消除。
    // 这里最终向上层暴露 3 路 16-bit 数据：Mic1、Mic3、Reference。具体通道重排见 Read()。
    input_reference_ = true;
    input_channels_ = input_channels;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // 30 dB 参考 esp-skainet Korvo-1 BSP 的 RECORD_VOLUME。适用于板载 3 麦语音唤醒/对话。
    // 可选值通常是 0~42 dB 左右，取决于 esp_codec_dev/ES7210 驱动限制；现场噪声大可适当降低，
    // 拾音距离远可适当升高，但过高会增加底噪和削波风险。
    input_gain_ = 30;

    CreateOutputChannel(output_mclk, output_bclk, output_ws, output_dout);
    CreateInputChannel(input_mclk, input_bclk, input_ws, input_din);

    audio_codec_i2s_cfg_t output_i2s_cfg = {
        // Korvo-1 硬件把 ES8311 播放接在 I2S0；这是 esp-skainet BSP 的配置依据。
        // 如果移植到复用同一条 I2S 的板子，可改为 I2S_NUM_0 单总线双向；Korvo-1 不适用。
        .port = I2S_NUM_0,
        .rx_handle = nullptr,
        .tx_handle = tx_handle_,
    };
    output_data_if_ = audio_codec_new_i2s_data(&output_i2s_cfg);
    assert(output_data_if_ != nullptr);

    audio_codec_i2s_cfg_t input_i2s_cfg = {
        // Korvo-1 硬件把 ES7210 录音接在 I2S1，和播放 I2S0 分开，便于全双工。
        // 可选值是 SoC 支持的 I2S_NUM_0/I2S_NUM_1；必须和实际引脚连线、CreateInputChannel() 一致。
        .port = I2S_NUM_1,
        .rx_handle = rx_handle_,
        .tx_handle = nullptr,
    };
    input_data_if_ = audio_codec_new_i2s_data(&input_i2s_cfg);
    assert(input_data_if_ != nullptr);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = AUDIO_CODEC_I2C_PORT,
        .addr = es8311_addr,
        .bus_handle = i2c_master_handle,
    };
    output_ctrl_if_ = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(output_ctrl_if_ != nullptr);

    gpio_if_ = audio_codec_new_gpio();
    assert(gpio_if_ != nullptr);

    es8311_codec_cfg_t es8311_cfg = {};
    es8311_cfg.ctrl_if = output_ctrl_if_;
    es8311_cfg.gpio_if = gpio_if_;
    es8311_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;
    es8311_cfg.pa_pin = pa_pin;

    // Korvo-1 的 ES8311 在官方 esp-skainet BSP 中配置为 use_mclk=false。
    // 适用于 codec 不需要由驱动额外打开 MCLK 的板级连接；若自制板 ES8311 依赖 MCLK，可改为 true。
    es8311_cfg.use_mclk = false;

    // 这两个电压用于 esp_codec_dev 估算硬件增益，沿用 ES8311 常见 5V PA、3.3V codec 供电场景。
    // 若 PA 或 codec 供电不同，应按硬件原理图调整，否则音量曲线可能不准。
    es8311_cfg.hw_gain.pa_voltage = 5.0;
    es8311_cfg.hw_gain.codec_dac_voltage = 3.3;
    output_codec_if_ = es8311_codec_new(&es8311_cfg);
    assert(output_codec_if_ != nullptr);

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = output_codec_if_,
        .data_if = output_data_if_,
    };
    output_dev_ = esp_codec_dev_new(&dev_cfg);
    assert(output_dev_ != nullptr);

    i2c_cfg.addr = es7210_addr;
    input_ctrl_if_ = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(input_ctrl_if_ != nullptr);

    es7210_codec_cfg_t es7210_cfg = {};
    es7210_cfg.ctrl_if = input_ctrl_if_;

    // ES7210 有 4 路 ADC。Korvo-1 板载 3 麦，另一路用作参考/回采通道，esp-skainet BSP 也选择 1~4 全开。
    // 如果硬件只接 1/2 路麦，可只选择 ES7210_SEL_MIC1/2；如果没有参考通道，需同步关闭 input_reference_。
    es7210_cfg.mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4;
    input_codec_if_ = es7210_codec_new(&es7210_cfg);
    assert(input_codec_if_ != nullptr);

    dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_IN;
    dev_cfg.codec_if = input_codec_if_;
    dev_cfg.data_if = input_data_if_;
    input_dev_ = esp_codec_dev_new(&dev_cfg);
    assert(input_dev_ != nullptr);

    ESP_LOGI(TAG, "Korvo1 audio codec initialized");
}

Korvo1AudioCodec::~Korvo1AudioCodec() {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_codec_dev_close(output_dev_));
    esp_codec_dev_delete(output_dev_);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_codec_dev_close(input_dev_));
    esp_codec_dev_delete(input_dev_);

    audio_codec_delete_codec_if(input_codec_if_);
    audio_codec_delete_ctrl_if(input_ctrl_if_);
    audio_codec_delete_codec_if(output_codec_if_);
    audio_codec_delete_ctrl_if(output_ctrl_if_);
    audio_codec_delete_gpio_if(gpio_if_);
    audio_codec_delete_data_if(input_data_if_);
    audio_codec_delete_data_if(output_data_if_);
}

void Korvo1AudioCodec::CreateOutputChannel(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout) {
    i2s_chan_config_t chan_cfg = {
        // I2S0 专用于 ES8311 播放输出；和 esp-skainet 的 Korvo-1 BSP 保持一致。
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,

        // DMA 参数沿用项目通用 AUDIO_CODEC_* 宏，适合 16 kHz 语音提示/对话播放。
        // 若改成高采样率音乐播放，可增大 frame/desc 来降低 underrun 风险，但会占用更多内存。
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,

            // 256 是 16-bit/32-bit I2S codec 常见 MCLK 倍频。可选 128/256/384/512 等，
            // 但要和 codec、采样率、位宽兼容；Korvo-1 按默认 256 即可。
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            // 播放链路使用 16-bit PCM，匹配项目内部音频格式和语音 TTS/提示音场景。
            // 可选 24/32-bit，但上层 buffer 和 codec open 参数也要一起改。
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,

            // ES8311 按标准 I2S 双声道时隙接收；上层实际可只写 mono，驱动会按 codec_dev 配置处理。
            // 可选 MONO/STEREO；若改 MONO，要确认 ES8311 和功放输出声道符合硬件连线。
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = mclk,
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_));
}

void Korvo1AudioCodec::CreateInputChannel(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t din) {
    i2s_chan_config_t chan_cfg = {
        // I2S1 专用于 ES7210 录音输入；Korvo-1 的输入和输出是两条 I2S，总线不能和 I2S0 混用。
        .id = I2S_NUM_1,
        .role = I2S_ROLE_MASTER,

        // 录音同样使用项目通用 DMA 参数，适合 16 kHz 连续语音采集。
        // 降低延迟可减小 frame，抗调度抖动可增大 frame/desc。
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)input_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,

            // ES7210 录音沿用 256 倍 MCLK。可选值同播放端，但必须保证 ES7210 能锁定时钟。
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            // Korvo-1 的 esp-skainet BSP 对 ES7210 输入使用 32-bit 标准 I2S、stereo slot。
            // 虽然最后交给 AFE 的是 int16_t，底层读数按 BSP 布局先取 4 路原始 16-bit 数据再重排。
            // 如果换成普通双麦/单麦 codec，常见可选值是 16-bit + MONO/STEREO。
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = mclk,
            .bclk = bclk,
            .ws = ws,
            .dout = I2S_GPIO_UNUSED,
            .din = din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_));
}

void Korvo1AudioCodec::SetOutputVolume(int volume) {
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(output_dev_, volume));
    AudioCodec::SetOutputVolume(volume);
}

void Korvo1AudioCodec::EnableInput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == input_enabled_) {
        return;
    }
    if (enable) {
        esp_codec_dev_sample_info_t fs = {
            // 这里必须和 CreateInputChannel() 的 32-bit/stereo 配置保持一致。
            // 依据是 Korvo-1 esp-skainet BSP: sample_rate=16000, channel=2, bits_per_sample=32。
            .bits_per_sample = 32,
            .channel = 2,
            .channel_mask = 0,
            .sample_rate = (uint32_t)input_sample_rate_,
            .mclk_multiple = 0,
        };
        ESP_ERROR_CHECK(esp_codec_dev_open(input_dev_, &fs));

        // ES7210 四路 ADC 增益：0/1/3 按麦克风通道给 30 dB，2 作为参考通道置 0 dB。
        // 这是从 esp-skainet Korvo-1 BSP 继承的通道约定；如果实测通道顺序不同，要同步改这里和 Read()。
        ESP_ERROR_CHECK(esp_codec_dev_set_in_channel_gain(input_dev_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), input_gain_));
        ESP_ERROR_CHECK(esp_codec_dev_set_in_channel_gain(input_dev_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1), input_gain_));
        ESP_ERROR_CHECK(esp_codec_dev_set_in_channel_gain(input_dev_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(2), 0.0));
        ESP_ERROR_CHECK(esp_codec_dev_set_in_channel_gain(input_dev_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(3), input_gain_));
    } else {
        ESP_ERROR_CHECK(esp_codec_dev_close(input_dev_));
    }
    AudioCodec::EnableInput(enable);
}

void Korvo1AudioCodec::EnableOutput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == output_enabled_) {
        return;
    }
    if (enable) {
        esp_codec_dev_sample_info_t fs = {
            // 项目播放链路统一使用 16-bit mono PCM；I2S 硬件仍配置 stereo slot 给 ES8311。
            // 若要做立体声音乐播放，可改 channel=2，并确认上层 Write() 输入也是双声道交织数据。
            .bits_per_sample = 16,
            .channel = 1,
            .channel_mask = 0,
            .sample_rate = (uint32_t)output_sample_rate_,
            .mclk_multiple = 0,
        };
        ESP_ERROR_CHECK(esp_codec_dev_open(output_dev_, &fs));
        ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(output_dev_, output_volume_));
    } else {
        ESP_ERROR_CHECK(esp_codec_dev_close(output_dev_));
    }
    AudioCodec::EnableOutput(enable);
}

int Korvo1AudioCodec::Read(int16_t* dest, int samples) {
    if (input_enabled_) {
        int frames = samples / input_channels_;

        // esp-skainet BSP 的 Korvo-1 原始采集布局按 4 路处理：
        //   raw[4*i + 0] = reference
        //   raw[4*i + 1] = mic
        //   raw[4*i + 2] = unused/noise/reference helper，当前不送入 AFE
        //   raw[4*i + 3] = mic
        // 本项目 AFE 需要的是连续 3 路 int16_t：Mic、Mic、Reference，因此这里做一次轻量重排。
        // 如果后续要启用更多麦克风或调整 AFE input_format，需要同步修改 input_channels_ 和这段映射。
        std::vector<int16_t> raw(frames * 4);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_codec_dev_read(input_dev_, raw.data(), raw.size() * sizeof(int16_t)));
        for (int i = 0; i < frames; ++i) {
            int16_t ref = raw[4 * i + 0];
            dest[3 * i + 0] = raw[4 * i + 1];
            dest[3 * i + 1] = raw[4 * i + 3];
            dest[3 * i + 2] = ref;
        }
    }
    return samples;
}

int Korvo1AudioCodec::Write(const int16_t* data, int samples) {
    if (output_enabled_) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_codec_dev_write(output_dev_, (void*)data, samples * sizeof(int16_t)));
    }
    return samples;
}
