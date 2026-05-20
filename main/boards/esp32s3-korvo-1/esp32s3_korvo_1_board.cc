#include "wifi_board.h"
#include "korvo1_audio_codec.h"
#include "led/circular_strip.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <driver/i2c_master.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>

#include <algorithm>
#include <cmath>
#include <vector>

#define TAG "esp32s3_korvo_1"

typedef enum {
    BSP_ADC_BUTTON_REC,
    BSP_ADC_BUTTON_VOL_MODE,
    BSP_ADC_BUTTON_PLAY,
    BSP_ADC_BUTTON_SET,
    BSP_ADC_BUTTON_VOL_DOWN,
    BSP_ADC_BUTTON_VOL_UP,
    BSP_ADC_BUTTON_NUM
} bsp_adc_button_t;

class Esp32S3Korvo1Board : public WifiBoard {
private:
    Button boot_button_;
    Button* adc_button_[BSP_ADC_BUTTON_NUM];
    adc_oneshot_unit_handle_t bsp_adc_handle = NULL;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    CircularStrip led_;
    bool source_led_active_ = false;
    uint32_t last_source_ms_ = 0;
    uint32_t last_led_update_ms_ = 0;

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = AUDIO_CODEC_I2C_PORT,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeButtons() {
        button_adc_config_t adc_cfg = {};
        adc_cfg.unit_id = ADC_UNIT_1;
        adc_cfg.adc_channel = ADC_CHANNEL_7; // GPIO8
        const adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &bsp_adc_handle));
        adc_cfg.adc_handle = &bsp_adc_handle;

        adc_cfg.button_index = BSP_ADC_BUTTON_REC;
        adc_cfg.min = 2310; // middle is 2410mV
        adc_cfg.max = 2510;
        adc_button_[BSP_ADC_BUTTON_REC] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_VOL_MODE;
        adc_cfg.min = 1880; // middle is 1980mV
        adc_cfg.max = 2080;
        adc_button_[BSP_ADC_BUTTON_VOL_MODE] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_PLAY;
        adc_cfg.min = 1550; // middle is 1650mV
        adc_cfg.max = 1750;
        adc_button_[BSP_ADC_BUTTON_PLAY] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_SET;
        adc_cfg.min = 1015; // middle is 1115mV
        adc_cfg.max = 1215;
        adc_button_[BSP_ADC_BUTTON_SET] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_VOL_DOWN;
        adc_cfg.min = 720; // middle is 820mV
        adc_cfg.max = 920;
        adc_button_[BSP_ADC_BUTTON_VOL_DOWN] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_VOL_UP;
        adc_cfg.min = 280; // middle is 380mV
        adc_cfg.max = 480;
        adc_button_[BSP_ADC_BUTTON_VOL_UP] = new AdcButton(adc_cfg);

        auto volume_up_button = adc_button_[BSP_ADC_BUTTON_VOL_UP];
        volume_up_button->OnClick([this]() { ChangeVol(10); });
        volume_up_button->OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
        });

        auto volume_down_button = adc_button_[BSP_ADC_BUTTON_VOL_DOWN];
        volume_down_button->OnClick([this]() { ChangeVol(-10); });
        volume_down_button->OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
        });

        auto volume_mode_button = adc_button_[BSP_ADC_BUTTON_VOL_MODE];
        volume_mode_button->OnClick([this]() { ToggleAecMode(); });

        auto play_button = adc_button_[BSP_ADC_BUTTON_PLAY];
        play_button->OnClick([this]() {
            ESP_LOGI(TAG, " TODO %s:%d\n", __func__, __LINE__);
        });

        auto set_button = adc_button_[BSP_ADC_BUTTON_SET];
        set_button->OnClick([this]() {
            EnterWifiConfigMode();
        });

        auto rec_button = adc_button_[BSP_ADC_BUTTON_REC];
        rec_button->OnClick([this]() {
            Application::GetInstance().ToggleChatState();
        });

        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        boot_button_.OnLongPress([this]() {
            EnterWifiConfigMode();
        });
    }

    void ChangeVol(int val) {
        auto codec = GetAudioCodec();
        auto volume = codec->output_volume() + val;
        if (volume > 100) {
            volume = 100;
        }
        if (volume < 0) {
            volume = 0;
        }
        codec->SetOutputVolume(volume);
    }

    void ToggleAecMode() {
#if CONFIG_USE_DEVICE_AEC
        auto& app = Application::GetInstance();
        app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
#else
        ESP_LOGI(TAG, "Device AEC is disabled");
#endif
    }

    bool ShouldShowSourceDirection() {
        auto state = Application::GetInstance().GetDeviceState();
        return state == kDeviceStateListening || state == kDeviceStateAudioTesting;
    }

    int AngleToLedIndex(float angle_deg) {
        int offset = static_cast<int>(std::lround(angle_deg / 360.0f * BUILTIN_LED_NUM)) % BUILTIN_LED_NUM;
        int direction = MIC_ARRAY_LED_CLOCKWISE ? 1 : -1;
        int index = MIC_ARRAY_LED_FRONT_INDEX + direction * offset;
        index %= BUILTIN_LED_NUM;
        if (index < 0) {
            index += BUILTIN_LED_NUM;
        }
        ESP_LOGI(TAG, "Angle: %f, index: %d", angle_deg, index);
        return index;
    }

    StripColor ScaleColor(uint8_t red, uint8_t green, uint8_t blue, float scale) {
        scale = std::max(0.0f, std::min(scale, 1.0f));
        return StripColor {
            static_cast<uint8_t>(red * scale),
            static_cast<uint8_t>(green * scale),
            static_cast<uint8_t>(blue * scale),
        };
    }

    void ShowSourceDirection(const Korvo1MicArrayResult& result) {
        int head = AngleToLedIndex(result.angle_deg);
        float confidence = std::max(0.2f, std::min(result.confidence, 1.0f));
        std::vector<StripColor> colors(BUILTIN_LED_NUM);

        colors[head] = ScaleColor(MIC_ARRAY_LED_HEAD_BRIGHTNESS, MIC_ARRAY_LED_SIDE_BRIGHTNESS, 0, confidence);
        for (int distance = 1; distance <= 2; ++distance) {
            float side_scale = confidence / (distance + 1);
            int left = (head - distance + BUILTIN_LED_NUM) % BUILTIN_LED_NUM;
            int right = (head + distance) % BUILTIN_LED_NUM;
            colors[left] = ScaleColor(MIC_ARRAY_LED_SIDE_BRIGHTNESS, MIC_ARRAY_LED_SIDE_BRIGHTNESS, 0, side_scale);
            colors[right] = ScaleColor(MIC_ARRAY_LED_SIDE_BRIGHTNESS, MIC_ARRAY_LED_SIDE_BRIGHTNESS, 0, side_scale);
        }

        led_.SetMultiColors(colors);
        source_led_active_ = true;
        last_source_ms_ = result.timestamp_ms;
        last_led_update_ms_ = result.timestamp_ms;
    }

public:
    Esp32S3Korvo1Board() : boot_button_(BOOT_BUTTON_GPIO), led_(BUILTIN_LED_GPIO, BUILTIN_LED_NUM) {
        ESP_LOGI(TAG, "Initializing ESP32-S3-Korvo-1 board");
        InitializeI2c();
        InitializeButtons();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Korvo1AudioCodec audio_codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_I2S_GPIO_MCLK,
            AUDIO_OUTPUT_I2S_GPIO_BCLK,
            AUDIO_OUTPUT_I2S_GPIO_WS,
            AUDIO_OUTPUT_I2S_GPIO_DOUT,
            AUDIO_INPUT_I2S_GPIO_MCLK,
            AUDIO_INPUT_I2S_GPIO_BCLK,
            AUDIO_INPUT_I2S_GPIO_WS,
            AUDIO_INPUT_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_CHANNELS);
        return &audio_codec;
    }

    virtual Led* GetLed() override {
        return &led_;
    }

    virtual void OnAudioInputFrame(const int16_t* data, size_t samples, int channels, uint32_t timestamp_ms) override {
        (void)data;
        (void)samples;

        if (channels != AUDIO_INPUT_CHANNELS || !ShouldShowSourceDirection()) {
            if (source_led_active_) {
                source_led_active_ = false;
                led_.OnStateChanged();
            }
            auto codec = static_cast<Korvo1AudioCodec*>(GetAudioCodec());
            codec->ResetMicArray();
            return;
        }

        auto codec = static_cast<Korvo1AudioCodec*>(GetAudioCodec());
        auto result = codec->last_mic_array_result();
        if (result.active) {
            if (timestamp_ms - last_led_update_ms_ >= MIC_ARRAY_LED_UPDATE_INTERVAL_MS) {
                ShowSourceDirection(result);
            } else {
                source_led_active_ = true;
                last_source_ms_ = timestamp_ms;
            }
        } else if (source_led_active_ && timestamp_ms - last_source_ms_ >= MIC_ARRAY_LED_IDLE_TIMEOUT_MS) {
            source_led_active_ = false;
            led_.OnStateChanged();
        }
    }
};

DECLARE_BOARD(Esp32S3Korvo1Board);
