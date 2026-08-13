#pragma once
// ES8311 single-chip full-duplex audio codec (speaker DAC + mic ADC). Reusable driver in boards/common
// the ES8311 is ONE codec that does both playback and capture. A single I2S port in standard mode
// carries both directions on the shared MCLK/BCLK/WS wires, and one I2C bus configures the registers:
//    Output: ESP DOUT -> ES8311 DSDIN  -> DAC -> speaker (mono 16-bit)
//    Input:  mic -> ADC -> ES8311 ASDOUT -> ESP DIN      (mono 16-bit)
//
// Wraps the public esp_codec_dev component
// Requires esp_codec_dev / esp_driver_i2c / esp_driver_i2s in REQUIRES
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_err.h"

struct Es8311Config {
    i2c_port_t i2c_port;
    gpio_num_t pin_sda;
    gpio_num_t pin_scl;
    gpio_num_t pin_mclk;
    gpio_num_t pin_bclk;    // SCLK
    gpio_num_t pin_ws;      // LRCK
    gpio_num_t pin_din;     // ESP DIN :mic / ADC
    gpio_num_t pin_dout;    // ESP DOUT : speaker / DAC
    gpio_num_t pin_pa_en;   // external PA enable, or GPIO_NUM_NC if the board has none
    bool       pa_active_low;  // PA enable polarity (false: PA on when pin is high)
    uint8_t    es8311_addr; // 7-bit I2C address (typically 0x18)
    i2s_port_t i2s_port;    // I2S peripheral to use (default I2S_NUM_0 when zero-initialized)
    int        sample_rate; // e.g. 16000
    int        mic_gain;    // ADC gain in dB (e.g. 30)
    int        out_volume;  // 0..100
};

class Es8311Codec {
public:
    // Build I2C + full-duplex I2S + ES8311, then open the codec (output ready, ADC running).
    esp_err_t Init(const Es8311Config& cfg);

    // Microphone (input). The ADC runs continuously under full duplex; these only gate ReadPcm and do
    // not touch the I2S clock, so playback keeps running across StartMic/StopMic.
    esp_err_t StartMic();
    esp_err_t StopMic();
    bool      MicOn() const { return mic_on_; }
    // Read 'frames' mono 16-bit samples into dest. *got = samples actually read. Blocks until the I2S
    // DMA has that many frames (~frames/sample_rate seconds)
    esp_err_t ReadPcm(int16_t* dest, size_t frames, size_t* got);

    // Speaker (output, ready right after Init).
    // Write 'frames' mono 16-bit samples; blocks until the I2S DMA has room.
    esp_err_t WritePcm(const int16_t* data, size_t frames);
    esp_err_t SetVolume(uint8_t vol);        // 0..100
    esp_err_t SetMicGain(int gain_db);

    bool Ready() const { return codec_dev_ != nullptr; }
    int  SampleRate() const { return cfg_.sample_rate; }

private:
    esp_err_t InitI2c();
    esp_err_t InitDuplexI2s();
    esp_err_t InitCodec();
    esp_err_t OpenDevice();

    Es8311Config cfg_ = {};
    i2c_master_bus_handle_t      i2c_bus_   = nullptr;
    i2s_chan_handle_t            tx_handle_ = nullptr;  // standard mode, speaker
    i2s_chan_handle_t            rx_handle_ = nullptr;  // standard mode, mic (shares the port with TX)
    const audio_codec_data_if_t* data_if_   = nullptr;
    const audio_codec_gpio_if_t* gpio_if_   = nullptr;
    const audio_codec_ctrl_if_t* ctrl_if_   = nullptr;
    const audio_codec_if_t*      codec_if_  = nullptr;
    esp_codec_dev_handle_t       codec_dev_ = nullptr;  // single IN_OUT handle backing both directions
    bool mic_on_   = false;
    bool dev_open_ = false;
};
