// ES8311 single-chip full-duplex codec implementation.
// Builds full-duplex I2S (TX + RX both standard mode, sharing MCLK/BCLK/WS) + one ES8311, then opens a
// single IN_OUT esp_codec_dev handle. Output is ready after Init; input is toggled with StartMic/StopMic,
// which only gate ReadPcm -- the handle stays open so the DAC (and thus playback) is never disturbed.
#include "es8311_audio.h"

#include "esp_check.h"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "Es8311";
constexpr int kI2sDmaDescNum  = 6;
constexpr int kI2sDmaFrameNum = 240;
constexpr int kBits           = 16;

// TX and RX share one BCLK/WS, so both must use the same slot layout (2 slots x 16-bit, Philips). The
// speaker/mic PCM is mono; esp_codec_dev maps mono <-> the two physical slots via the channel=1 open.
void FillStd(i2s_std_config_t& std, int sample_rate,
             gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din) {
    std.clk_cfg.sample_rate_hz = sample_rate;
    std.clk_cfg.clk_src        = I2S_CLK_SRC_DEFAULT;
    std.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_256;
    std.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;
    std.slot_cfg.slot_mode      = I2S_SLOT_MODE_STEREO;
    std.slot_cfg.slot_mask      = I2S_STD_SLOT_BOTH;
    std.slot_cfg.ws_width       = I2S_DATA_BIT_WIDTH_16BIT;
    std.slot_cfg.ws_pol         = false;
    std.slot_cfg.bit_shift      = true;   // Philips
    std.slot_cfg.left_align     = true;
    std.slot_cfg.big_endian     = false;
    std.slot_cfg.bit_order_lsb  = false;
    std.gpio_cfg.mclk = mclk;
    std.gpio_cfg.bclk = bclk;
    std.gpio_cfg.ws   = ws;
    std.gpio_cfg.dout = dout;
    std.gpio_cfg.din  = din;
}
}  // namespace

esp_err_t Es8311Codec::Init(const Es8311Config& cfg) {
    cfg_ = cfg;
    ESP_RETURN_ON_ERROR(InitI2c(),       TAG, "i2c");
    ESP_RETURN_ON_ERROR(InitDuplexI2s(), TAG, "i2s");
    ESP_RETURN_ON_ERROR(InitCodec(),     TAG, "codec");
    ESP_RETURN_ON_ERROR(OpenDevice(),    TAG, "open");
    ESP_LOGI(TAG, "ES8311 ready (full-duplex I2S @ %d Hz, vol=%d, mic_gain=%d dB)",
             cfg_.sample_rate, cfg_.out_volume, cfg_.mic_gain);
    return ESP_OK;
}

esp_err_t Es8311Codec::InitI2c() {
    i2c_master_bus_config_t bus = {};
    bus.i2c_port          = cfg_.i2c_port;
    bus.sda_io_num        = cfg_.pin_sda;
    bus.scl_io_num        = cfg_.pin_scl;
    bus.clk_source        = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    bus.flags.enable_internal_pullup = true;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus, &i2c_bus_), TAG, "i2c bus");
    return ESP_OK;
}

esp_err_t Es8311Codec::InitDuplexI2s() {
    // Full duplex: create TX + RX together on one port so they share MCLK/BCLK/WS.
    i2s_chan_config_t chan = {};
    chan.id                  = cfg_.i2s_port;
    chan.role                = I2S_ROLE_MASTER;   // ESP drives the clocks; the ES8311 is the I2S slave
    chan.dma_desc_num        = kI2sDmaDescNum;
    chan.dma_frame_num       = kI2sDmaFrameNum;
    chan.auto_clear_after_cb = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan, &tx_handle_, &rx_handle_), TAG, "i2s new");

    // TX: speaker path drives dout, din unused.
    i2s_std_config_t tx_std = {};
    FillStd(tx_std, cfg_.sample_rate, cfg_.pin_mclk, cfg_.pin_bclk, cfg_.pin_ws,
            cfg_.pin_dout, I2S_GPIO_UNUSED);
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx_handle_, &tx_std), TAG, "tx std init");

    // RX: mic path reads din, dout unused.
    i2s_std_config_t rx_std = {};
    FillStd(rx_std, cfg_.sample_rate, cfg_.pin_mclk, cfg_.pin_bclk, cfg_.pin_ws,
            I2S_GPIO_UNUSED, cfg_.pin_din);
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_handle_, &rx_std), TAG, "rx std init");

    // Enable both so the clock runs continuously; capture/playback are gated by esp_codec_dev, not the clock.
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx_handle_), TAG, "en tx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_handle_), TAG, "en rx");
    return ESP_OK;
}

esp_err_t Es8311Codec::InitCodec() {
    audio_codec_i2s_cfg_t i2s_cfg = {};
    i2s_cfg.port      = cfg_.i2s_port;
    i2s_cfg.rx_handle = rx_handle_;
    i2s_cfg.tx_handle = tx_handle_;
    data_if_ = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if_, ESP_FAIL, TAG, "data_if");

    gpio_if_ = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if_, ESP_FAIL, TAG, "gpio_if");

    audio_codec_i2c_cfg_t i2c = {};
    i2c.port       = cfg_.i2c_port;
    i2c.addr       = cfg_.es8311_addr << 1;  // esp_codec_dev uses 8-bit addresses
    i2c.bus_handle = i2c_bus_;
    ctrl_if_ = audio_codec_new_i2c_ctrl(&i2c);
    ESP_RETURN_ON_FALSE(ctrl_if_, ESP_FAIL, TAG, "es8311 ctrl");

    // One codec configured for BOTH ADC and DAC -> a single chip serving mic and speaker.
    es8311_codec_cfg_t es8311 = {};
    es8311.ctrl_if     = ctrl_if_;
    es8311.gpio_if     = gpio_if_;
    es8311.codec_mode  = ESP_CODEC_DEV_WORK_MODE_BOTH;
    es8311.pa_pin      = cfg_.pin_pa_en;                 // GPIO_NUM_NC (-1) disables PA control
    es8311.pa_reverted = cfg_.pa_active_low;
    es8311.use_mclk    = true;
    es8311.hw_gain.pa_voltage        = 3.3f;
    es8311.hw_gain.codec_dac_voltage = 3.3f;
    codec_if_ = es8311_codec_new(&es8311);
    ESP_RETURN_ON_FALSE(codec_if_, ESP_FAIL, TAG, "es8311_new");

    // Single IN_OUT device: one open enables both directions; we never close it mid-session (see header).
    esp_codec_dev_cfg_t dev = {};
    dev.dev_type = ESP_CODEC_DEV_TYPE_IN_OUT;
    dev.codec_if = codec_if_;
    dev.data_if  = data_if_;
    codec_dev_ = esp_codec_dev_new(&dev);
    ESP_RETURN_ON_FALSE(codec_dev_, ESP_FAIL, TAG, "codec dev");
    return ESP_OK;
}

esp_err_t Es8311Codec::OpenDevice() {
    if (dev_open_) return ESP_OK;
    esp_codec_dev_sample_info_t fs = {};
    fs.bits_per_sample = kBits;
    fs.channel         = 1;   // mono both ways
    fs.channel_mask    = 0;
    fs.sample_rate     = cfg_.sample_rate;
    const int r = esp_codec_dev_open(codec_dev_, &fs);
    if (r != 0) { ESP_LOGE(TAG, "open codec=%d", r); return ESP_FAIL; }
    (void)esp_codec_dev_set_out_vol(codec_dev_, cfg_.out_volume);
    (void)esp_codec_dev_set_in_gain(codec_dev_, static_cast<float>(cfg_.mic_gain));
    dev_open_ = true;
    return ESP_OK;
}

esp_err_t Es8311Codec::StartMic() {
    if (!dev_open_) return ESP_ERR_INVALID_STATE;
    if (mic_on_) return ESP_OK;
    mic_on_ = true;
    ESP_LOGD(TAG, "mic on (gain=%d dB)", cfg_.mic_gain);
    return ESP_OK;
}

esp_err_t Es8311Codec::StopMic() {
    if (!mic_on_) return ESP_OK;
    mic_on_ = false;  // full duplex: leave the device open so the DAC/clock keep running
    ESP_LOGD(TAG, "mic off");
    return ESP_OK;
}

esp_err_t Es8311Codec::ReadPcm(int16_t* dest, size_t frames, size_t* got) {
    if (got) *got = 0;
    if (!dest || !mic_on_) return ESP_ERR_INVALID_STATE;
    const int r = esp_codec_dev_read(codec_dev_, dest, frames * sizeof(int16_t));  // channel=1 -> mono
    if (r != 0) return ESP_FAIL;
    if (got) *got = frames;
    return ESP_OK;
}

esp_err_t Es8311Codec::WritePcm(const int16_t* data, size_t frames) {
    if (!data || !dev_open_) return ESP_ERR_INVALID_STATE;
    const int r = esp_codec_dev_write(codec_dev_, const_cast<int16_t*>(data), frames * sizeof(int16_t));
    return (r == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t Es8311Codec::SetVolume(uint8_t vol) {
    cfg_.out_volume = (vol > 100) ? 100 : vol;
    if (dev_open_) (void)esp_codec_dev_set_out_vol(codec_dev_, cfg_.out_volume);
    return ESP_OK;
}

esp_err_t Es8311Codec::SetMicGain(int gain_db) {
    cfg_.mic_gain = gain_db;
    if (dev_open_) (void)esp_codec_dev_set_in_gain(codec_dev_, static_cast<float>(gain_db));
    return ESP_OK;
}
