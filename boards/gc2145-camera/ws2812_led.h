#pragma once
#include <cstdint>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

// Minimal driver for a single onboard WS2812 / NeoPixel RGB LED, driven over one RMT TX channel.
// SetColor latches immediately: after the 3 GRB bytes the line idles low, which is the WS2812 reset/latch.
// No external dependency (uses the core esp_driver_rmt), so it builds offline — matching this board's
// self-contained St7789Lcd driver rather than pulling in the led_strip managed component.
class Ws2812Led {
public:
    esp_err_t Init(gpio_num_t gpio);
    esp_err_t SetColor(uint8_t r, uint8_t g, uint8_t b);
    esp_err_t SetRgb(uint32_t rgb) { return SetColor((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF); } // 0x00RRGGBB
    esp_err_t Off() { return SetColor(0, 0, 0); }
    bool Ready() const { return chan_ && enc_; }

private:
    rmt_channel_handle_t chan_ = nullptr;
    rmt_encoder_handle_t enc_  = nullptr;
};
