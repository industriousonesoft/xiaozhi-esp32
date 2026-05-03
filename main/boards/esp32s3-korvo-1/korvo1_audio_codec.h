#ifndef _KORVO1_AUDIO_CODEC_H_
#define _KORVO1_AUDIO_CODEC_H_

#include "audio_codec.h"

#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>

#include <mutex>

class Korvo1AudioCodec : public AudioCodec {
private:
    const audio_codec_data_if_t* output_data_if_ = nullptr;
    const audio_codec_data_if_t* input_data_if_ = nullptr;
    const audio_codec_ctrl_if_t* output_ctrl_if_ = nullptr;
    const audio_codec_ctrl_if_t* input_ctrl_if_ = nullptr;
    const audio_codec_if_t* output_codec_if_ = nullptr;
    const audio_codec_if_t* input_codec_if_ = nullptr;
    const audio_codec_gpio_if_t* gpio_if_ = nullptr;
    esp_codec_dev_handle_t output_dev_ = nullptr;
    esp_codec_dev_handle_t input_dev_ = nullptr;
    std::mutex data_if_mutex_;

    void CreateOutputChannel(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout);
    void CreateInputChannel(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t din);

    virtual int Read(int16_t* dest, int samples) override;
    virtual int Write(const int16_t* data, int samples) override;

public:
    Korvo1AudioCodec(void* i2c_master_handle, int input_sample_rate, int output_sample_rate,
        gpio_num_t output_mclk, gpio_num_t output_bclk, gpio_num_t output_ws, gpio_num_t output_dout,
        gpio_num_t input_mclk, gpio_num_t input_bclk, gpio_num_t input_ws, gpio_num_t input_din,
        gpio_num_t pa_pin, uint8_t es8311_addr, uint8_t es7210_addr, int input_channels);
    virtual ~Korvo1AudioCodec();

    virtual void SetOutputVolume(int volume) override;
    virtual void EnableInput(bool enable) override;
    virtual void EnableOutput(bool enable) override;
};

#endif // _KORVO1_AUDIO_CODEC_H_
