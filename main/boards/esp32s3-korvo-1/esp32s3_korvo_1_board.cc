#include "wifi_board.h"
#include "korvo1_audio_codec.h"
#include "led/circular_strip.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "assets/lang_config.h"

#include <driver/i2c_master.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_idf_version.h>
#include <esp_log.h>

#include <algorithm>
#include <cmath>
#include <vector>

#define TAG "esp32s3_korvo_1"

typedef enum {
    BSP_ADC_BUTTON_REC,
    BSP_ADC_BUTTON_MODE,
    BSP_ADC_BUTTON_PLAY,
    BSP_ADC_BUTTON_SET,
    BSP_ADC_BUTTON_VOL_DOWN,
    BSP_ADC_BUTTON_VOL_UP,
    BSP_ADC_BUTTON_NUM
} bsp_adc_button_t;

class Esp32S3Korvo1Board : public WifiBoard {
private:
    Button boot_button_;
    Button* adc_button_[BSP_ADC_BUTTON_NUM] = {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
#endif
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

    void ChangeVolume(int delta) {
        auto codec = GetAudioCodec();
        int volume = codec->output_volume() + delta;
        volume = std::max(0, std::min(volume, 100));
        codec->SetOutputVolume(volume);
        GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
    }

    void InitializeButtons() {
        button_adc_config_t adc_cfg = {};
        adc_cfg.unit_id = ADC_UNIT_1;
        adc_cfg.adc_channel = ADC_CHANNEL_7; // GPIO8
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        const adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));
        adc_cfg.adc_handle = &adc_handle_;
#endif
        adc_cfg.button_index = BSP_ADC_BUTTON_REC;
        adc_cfg.min = 2310;
        adc_cfg.max = 2510;
        adc_button_[BSP_ADC_BUTTON_REC] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_MODE;
        adc_cfg.min = 1880;
        adc_cfg.max = 2080;
        adc_button_[BSP_ADC_BUTTON_MODE] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_PLAY;
        adc_cfg.min = 1550;
        adc_cfg.max = 1750;
        adc_button_[BSP_ADC_BUTTON_PLAY] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_SET;
        adc_cfg.min = 1015;
        adc_cfg.max = 1215;
        adc_button_[BSP_ADC_BUTTON_SET] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_VOL_DOWN;
        adc_cfg.min = 720;
        adc_cfg.max = 920;
        adc_button_[BSP_ADC_BUTTON_VOL_DOWN] = new AdcButton(adc_cfg);

        adc_cfg.button_index = BSP_ADC_BUTTON_VOL_UP;
        adc_cfg.min = 280;
        adc_cfg.max = 480;
        adc_button_[BSP_ADC_BUTTON_VOL_UP] = new AdcButton(adc_cfg);

        auto volume_up_button = adc_button_[BSP_ADC_BUTTON_VOL_UP];
        volume_up_button->OnClick([this]() {
            ChangeVolume(10);
        });
        volume_up_button->OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        auto volume_down_button = adc_button_[BSP_ADC_BUTTON_VOL_DOWN];
        volume_down_button->OnClick([this]() {
            ChangeVolume(-10);
        });
        volume_down_button->OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });

        auto set_button = adc_button_[BSP_ADC_BUTTON_SET];
        set_button->OnClick([this]() {
            auto& app = Application::GetInstance();
            app.ToggleAfeLocalPlaybackMode();
            bool enabled = app.GetAudioService().IsAfeLocalPlaybackEnabled();
            ESP_LOGI(TAG, "AFE local playback test mode: %s", enabled ? "enabled" : "disabled");
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
