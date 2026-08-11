#pragma once
// ES8311 (speaker out) + ES7210 (mic in) full-duplex codec. Reusable driver in boards/common.
//
// Wraps the public esp_codec_dev component: I2C configures the codec registers, I2S moves PCM full-duplex.
//   - Output: I2S standard mode -> ES8311 -> speaker (mono 16-bit)
//   - Input:  I2S TDM 4-slot -> ES7210 -> read slot0 (MIC1) for mono 16-bit PCM
// TX and RX share one I2S port and the same MCLK/BCLK/WS wires, so they must be created together (full
// duplex); they switch independently: recording (StartMic/StopMic) and playback (WritePcm, output always
// on) do not affect each other.
//
// Requires esp_codec_dev / esp_driver_i2c / esp_driver_i2s in REQUIRES.
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_err.h"

struct EsCodecConfig {
    i2c_port_t i2c_port;
    gpio_num_t pin_sda;
    gpio_num_t pin_scl;
    gpio_num_t pin_mclk;
    gpio_num_t pin_bclk;
    gpio_num_t pin_ws;
    gpio_num_t pin_din;      // mic
    gpio_num_t pin_dout;     // speaker
    gpio_num_t pin_pa_en;    // PA enable
    uint8_t    es7210_addr;
    uint8_t    es8311_addr;
    int        sample_rate;
    int        mic_gain;
    int        out_volume;
};

class EsCodec {
public:
    esp_err_t Init(const EsCodecConfig& cfg);  // build I2C + full-duplex I2S + ES7210/ES8311, and open the output

    // Microphone (input)
    esp_err_t StartMic();   // start capture
    esp_err_t StopMic();    // stop capture; does not stop the I2S clock under full duplex, output stays on)
    bool      MicOn() const { return mic_on_; }
    // Read 'frames' mono 16-bit samples into dest. *got = samples actually read
    esp_err_t ReadPcm(int16_t* dest, size_t frames, size_t* got);

    // Speaker (output, always on after Init)
    // Write 'frames' mono 16-bit samples; blocks until the I2S DMA has room (do NOT call from the BLE host task)
    esp_err_t WritePcm(const int16_t* data, size_t frames);
    esp_err_t SetVolume(uint8_t vol);

    bool Ready() const { return output_dev_ != nullptr; }
    int  SampleRate() const { return cfg_.sample_rate; }

private:
    esp_err_t InitI2c();
    esp_err_t InitDuplexI2s();
    esp_err_t InitCodecs();
    esp_err_t OpenOutput();

    EsCodecConfig cfg_ = {};
    i2c_master_bus_handle_t      i2c_bus_    = nullptr;
    i2s_chan_handle_t            tx_handle_  = nullptr;  // standard mode, speaker
    i2s_chan_handle_t            rx_handle_  = nullptr;  // TDM mode, microphone
    const audio_codec_data_if_t* data_if_    = nullptr;
    const audio_codec_gpio_if_t* gpio_if_    = nullptr;
    const audio_codec_ctrl_if_t* out_ctrl_if_= nullptr;
    const audio_codec_ctrl_if_t* in_ctrl_if_ = nullptr;
    const audio_codec_if_t*      out_codec_if_= nullptr;
    const audio_codec_if_t*      in_codec_if_= nullptr;
    esp_codec_dev_handle_t       output_dev_ = nullptr;
    esp_codec_dev_handle_t       input_dev_  = nullptr;
    bool mic_on_        = false;
    bool output_opened_ = false;
};
