// ES8311 + ES7210 full-duplex codec implementation
// Builds full-duplex I2S (TX standard + RX TDM) + both codecs; the output stays open after Init,
// and input is toggled with StartMic/StopMic
#include "es_codec.h"

#include "esp_check.h"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "EsCodec";
constexpr int kI2sDmaDescNum  = 6;
constexpr int kI2sDmaFrameNum = 240;
constexpr int kBits           = 16;
}

esp_err_t EsCodec::Init(const EsCodecConfig& cfg) {
    cfg_ = cfg;
    ESP_RETURN_ON_ERROR(InitI2c(),        TAG, "i2c");
    ESP_RETURN_ON_ERROR(InitDuplexI2s(),  TAG, "i2s");
    ESP_RETURN_ON_ERROR(InitCodecs(),     TAG, "codec");
    ESP_RETURN_ON_ERROR(OpenOutput(),     TAG, "open output");
    ESP_LOGI(TAG, "ES8311+ES7210 ready (duplex I2S @ %d Hz)", cfg_.sample_rate);
    return ESP_OK;
}

esp_err_t EsCodec::InitI2c() {
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

esp_err_t EsCodec::InitDuplexI2s() {
    // Full duplex: create TX (standard, speaker) + RX (TDM, 4 mics) together, sharing MCLK/BCLK/WS
    i2s_chan_config_t chan = {};
    chan.id                  = I2S_NUM_0;
    chan.role                = I2S_ROLE_MASTER;
    chan.dma_desc_num        = kI2sDmaDescNum;
    chan.dma_frame_num       = kI2sDmaFrameNum;
    chan.auto_clear_after_cb = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan, &tx_handle_, &rx_handle_), TAG, "i2s new");

    // TX: standard 16-bit stereo
    i2s_std_config_t std = {};
    std.clk_cfg.sample_rate_hz = cfg_.sample_rate;
    std.clk_cfg.clk_src        = I2S_CLK_SRC_DEFAULT;
    std.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_256;
    std.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;
    std.slot_cfg.slot_mode      = I2S_SLOT_MODE_STEREO;
    std.slot_cfg.slot_mask      = I2S_STD_SLOT_BOTH;
    std.slot_cfg.ws_width       = I2S_DATA_BIT_WIDTH_16BIT;
    std.slot_cfg.ws_pol         = false;
    std.slot_cfg.bit_shift      = true;
    std.slot_cfg.left_align     = true;
    std.slot_cfg.big_endian     = false;
    std.slot_cfg.bit_order_lsb  = false;
    std.gpio_cfg.mclk = cfg_.pin_mclk;
    std.gpio_cfg.bclk = cfg_.pin_bclk;
    std.gpio_cfg.ws   = cfg_.pin_ws;
    std.gpio_cfg.dout = cfg_.pin_dout;
    std.gpio_cfg.din  = I2S_GPIO_UNUSED;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx_handle_, &std), TAG, "std init");

    // RX: TDM 16-bit 4-slot
    i2s_tdm_config_t tdm = {};
    tdm.clk_cfg.sample_rate_hz = cfg_.sample_rate;
    tdm.clk_cfg.clk_src        = I2S_CLK_SRC_DEFAULT;
    tdm.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_256;
    tdm.clk_cfg.bclk_div       = 8;
    tdm.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    tdm.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;
    tdm.slot_cfg.slot_mode      = I2S_SLOT_MODE_STEREO;
    tdm.slot_cfg.slot_mask      = static_cast<i2s_tdm_slot_mask_t>(
        I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3);
    tdm.slot_cfg.ws_width       = I2S_TDM_AUTO_WS_WIDTH;
    tdm.slot_cfg.ws_pol         = false;
    tdm.slot_cfg.bit_shift      = true;
    tdm.slot_cfg.left_align     = false;
    tdm.slot_cfg.big_endian     = false;
    tdm.slot_cfg.bit_order_lsb  = false;
    tdm.slot_cfg.skip_mask      = false;
    tdm.slot_cfg.total_slot     = I2S_TDM_AUTO_SLOT_NUM;
    tdm.gpio_cfg.mclk = cfg_.pin_mclk;
    tdm.gpio_cfg.bclk = cfg_.pin_bclk;
    tdm.gpio_cfg.ws   = cfg_.pin_ws;
    tdm.gpio_cfg.dout = I2S_GPIO_UNUSED;
    tdm.gpio_cfg.din  = cfg_.pin_din;
    ESP_RETURN_ON_ERROR(i2s_channel_init_tdm_mode(rx_handle_, &tdm), TAG, "tdm init");

    // Enable both channels so the clock runs continuously; capture/playback are gated by esp_codec_dev open/close.
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx_handle_), TAG, "en tx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_handle_), TAG, "en rx");
    return ESP_OK;
}

esp_err_t EsCodec::InitCodecs() {
    audio_codec_i2s_cfg_t i2s_cfg = {};
    i2s_cfg.port      = I2S_NUM_0;
    i2s_cfg.rx_handle = rx_handle_;
    i2s_cfg.tx_handle = tx_handle_;
    data_if_ = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if_, ESP_FAIL, TAG, "data_if");

    gpio_if_ = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if_, ESP_FAIL, TAG, "gpio_if");

    // ── ES8311 output ──
    audio_codec_i2c_cfg_t o_i2c = {};
    o_i2c.port = cfg_.i2c_port;
    o_i2c.addr = cfg_.es8311_addr << 1;  // esp_codec_dev uses 8-bit addresses
    o_i2c.bus_handle = i2c_bus_;
    out_ctrl_if_ = audio_codec_new_i2c_ctrl(&o_i2c);
    ESP_RETURN_ON_FALSE(out_ctrl_if_, ESP_FAIL, TAG, "es8311 ctrl");

    es8311_codec_cfg_t es8311 = {};
    es8311.ctrl_if    = out_ctrl_if_;
    es8311.gpio_if    = gpio_if_;
    es8311.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;
    es8311.pa_pin     = cfg_.pin_pa_en;
    es8311.use_mclk   = true;
    es8311.hw_gain.pa_voltage        = 3.3f;
    es8311.hw_gain.codec_dac_voltage = 3.3f;
    out_codec_if_ = es8311_codec_new(&es8311);
    ESP_RETURN_ON_FALSE(out_codec_if_, ESP_FAIL, TAG, "es8311_new");

    esp_codec_dev_cfg_t o_dev = {};
    o_dev.dev_type = ESP_CODEC_DEV_TYPE_OUT;
    o_dev.codec_if = out_codec_if_;
    o_dev.data_if  = data_if_;
    output_dev_ = esp_codec_dev_new(&o_dev);
    ESP_RETURN_ON_FALSE(output_dev_, ESP_FAIL, TAG, "out dev");

    // ── ES7210 input ──
    audio_codec_i2c_cfg_t i_i2c = {};
    i_i2c.port = cfg_.i2c_port;
    i_i2c.addr = cfg_.es7210_addr << 1;
    i_i2c.bus_handle = i2c_bus_;
    in_ctrl_if_ = audio_codec_new_i2c_ctrl(&i_i2c);
    ESP_RETURN_ON_FALSE(in_ctrl_if_, ESP_FAIL, TAG, "es7210 ctrl");

    es7210_codec_cfg_t es7210 = {};
    es7210.ctrl_if      = in_ctrl_if_;
    es7210.mic_selected = 0x0F;  // all 4 mics on; on read, channel_mask picks slot0
    in_codec_if_ = es7210_codec_new(&es7210);
    ESP_RETURN_ON_FALSE(in_codec_if_, ESP_FAIL, TAG, "es7210_new");

    esp_codec_dev_cfg_t i_dev = {};
    i_dev.dev_type = ESP_CODEC_DEV_TYPE_IN;
    i_dev.codec_if = in_codec_if_;
    i_dev.data_if  = data_if_;
    input_dev_ = esp_codec_dev_new(&i_dev);
    ESP_RETURN_ON_FALSE(input_dev_, ESP_FAIL, TAG, "in dev");
    return ESP_OK;
}

esp_err_t EsCodec::OpenOutput() {
    if (output_opened_) return ESP_OK;
    esp_codec_dev_sample_info_t fs = {};
    fs.bits_per_sample = kBits;
    fs.channel         = 1;   // mono
    fs.channel_mask    = 0;
    fs.sample_rate     = cfg_.sample_rate;
    const int r = esp_codec_dev_open(output_dev_, &fs);
    if (r != 0) { ESP_LOGE(TAG, "open output=%d", r); return ESP_FAIL; }
    (void)esp_codec_dev_set_out_vol(output_dev_, cfg_.out_volume);
    output_opened_ = true;
    ESP_LOGI(TAG, "ES8311 output opened (vol=%d)", cfg_.out_volume);
    return ESP_OK;
}

esp_err_t EsCodec::StartMic() {
    if (!input_dev_) return ESP_ERR_INVALID_STATE;
    if (mic_on_) return ESP_OK;
    esp_codec_dev_sample_info_t fs = {};
    fs.bits_per_sample = kBits;
    fs.channel         = 4;   // TDM 4 slots
    fs.channel_mask    = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0);  // take only MIC1 (slot0)
    fs.sample_rate     = cfg_.sample_rate;
    const int r = esp_codec_dev_open(input_dev_, &fs);
    if (r != 0) { ESP_LOGE(TAG, "open input=%d", r); return ESP_FAIL; }
    (void)esp_codec_dev_set_in_channel_gain(
        input_dev_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), static_cast<float>(cfg_.mic_gain));
    mic_on_ = true;
    ESP_LOGI(TAG, "mic started (gain=%d)", cfg_.mic_gain);
    return ESP_OK;
}

esp_err_t EsCodec::StopMic() {
    if (!input_dev_ || !mic_on_) return ESP_OK;
    (void)esp_codec_dev_close(input_dev_);  // full duplex: output stays open -> the I2S clock keeps running, so the RX channel isn't disabled
    mic_on_ = false;
    ESP_LOGI(TAG, "mic stopped");
    return ESP_OK;
}

esp_err_t EsCodec::ReadPcm(int16_t* dest, size_t frames, size_t* got) {
    if (got) *got = 0;
    if (!dest || !mic_on_) return ESP_ERR_INVALID_STATE;
    const int r = esp_codec_dev_read(input_dev_, dest, frames * sizeof(int16_t));  // mask = 1 channel -> mono
    if (r != 0) return ESP_FAIL;
    if (got) *got = frames;
    return ESP_OK;
}

esp_err_t EsCodec::WritePcm(const int16_t* data, size_t frames) {
    if (!data || !output_opened_) return ESP_ERR_INVALID_STATE;
    const int r = esp_codec_dev_write(output_dev_, const_cast<int16_t*>(data), frames * sizeof(int16_t));
    return (r == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t EsCodec::SetVolume(uint8_t vol) {
    cfg_.out_volume = (vol > 100) ? 100 : vol;
    if (output_opened_) (void)esp_codec_dev_set_out_vol(output_dev_, cfg_.out_volume);
    return ESP_OK;
}
