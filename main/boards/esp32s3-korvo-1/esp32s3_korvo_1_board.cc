#include "wifi_board.h"
#include "korvo1_audio_codec.h"
#include "led/circular_strip.h"
#include "application.h"
#include "button.h"
#include "config.h"

#include <driver/i2c_master.h>
#include <esp_log.h>

#define TAG "esp32s3_korvo_1"

class Esp32S3Korvo1Board : public WifiBoard {
private:
    Button boot_button_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;

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

public:
    Esp32S3Korvo1Board() : boot_button_(BOOT_BUTTON_GPIO) {
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
        static CircularStrip led(BUILTIN_LED_GPIO, BUILTIN_LED_NUM);
        return &led;
    }
};

DECLARE_BOARD(Esp32S3Korvo1Board);
