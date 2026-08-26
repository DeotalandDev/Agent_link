#include "ws2812_led.h"

#include "esp_check.h"

namespace {
constexpr const char* TAG = "ws2812";
constexpr uint32_t kResolutionHz = 10 * 1000 * 1000;  // 0.1 us per RMT tick
}  // namespace

esp_err_t Ws2812Led::Init(gpio_num_t gpio) {
    rmt_tx_channel_config_t tx = {};
    tx.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx.gpio_num          = gpio;
    tx.mem_block_symbols = 64;
    tx.resolution_hz     = kResolutionHz;
    tx.trans_queue_depth = 4;
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx, &chan_), TAG, "new_tx_channel");

    // WS2812 bit timings @10MHz (0.1us/tick): '0' = 0.3us high + 0.9us low; '1' = 0.9us high + 0.3us low.
    rmt_bytes_encoder_config_t enc = {};
    enc.bit0.level0 = 1; enc.bit0.duration0 = 3;
    enc.bit0.level1 = 0; enc.bit0.duration1 = 9;
    enc.bit1.level0 = 1; enc.bit1.duration0 = 9;
    enc.bit1.level1 = 0; enc.bit1.duration1 = 3;
    enc.flags.msb_first = 1;   // each byte is shifted out MSB first
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&enc, &enc_), TAG, "new_bytes_encoder");

    ESP_RETURN_ON_ERROR(rmt_enable(chan_), TAG, "enable");
    return ESP_OK;
}

esp_err_t Ws2812Led::SetColor(uint8_t r, uint8_t g, uint8_t b) {
    if (!Ready()) return ESP_ERR_INVALID_STATE;
    const uint8_t grb[3] = { g, r, b };   // WS2812 wire order is G, R, B
    rmt_transmit_config_t txc = {};       // loop_count=0, eot_level=0 -> line idles low after TX = the latch/reset
    ESP_RETURN_ON_ERROR(rmt_transmit(chan_, enc_, grb, sizeof grb, &txc), TAG, "transmit");
    return rmt_tx_wait_all_done(chan_, 100);
}
