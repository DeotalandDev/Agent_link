#pragma once
// ═══════════════════════════════════════════════════════════════════════════════
// Co5300Panel: reusable driver for the CO5300 466x466 round AMOLED (MIPI-DSI), in boards/common.
//
// For ESP32-P4: LDO (2.5V for the D-PHY) -> DSI bus -> DBI command port -> CO5300 panel -> draw_bitmap.
// The board only supplies the reset pin + resolution; the rest (lane count / bitrate / init sequence)
// is handled by the official esp_lcd_co5300 component.
//
// Note: this header deliberately includes no MIPI headers; handles are stored as void* (cast to the
//   real types in the .cc). That keeps it chip-independent, so it compiles fine even though
//   boards/common is GLOB-compiled for every board; the real MIPI implementation lives only in
//   co5300_panel.cc, gated on CONFIG_IDF_TARGET_ESP32P4.
//
// Reference implementation: lights up + solid-color fill + brightness, enough for a boot self-test
// or a ShowText background. Animation / double buffering are out of scope.
// ═══════════════════════════════════════════════════════════════════════════════
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

struct Co5300Config {
    int      rst_gpio;    // reset GPIO (CO5300 active-low hardware reset); -1 = not connected
    int      pwr_en_gpio; // panel power-enable GPIO (active high, e.g. VCI_EN); -1 = none (always powered)
    uint16_t width;       // 466
    uint16_t height;      // 466
};

class Co5300Panel {
public:
    Co5300Panel() = default;
    ~Co5300Panel();

    Co5300Panel(const Co5300Panel&) = delete;
    Co5300Panel& operator=(const Co5300Panel&) = delete;

    // Bring up LDO + DSI + DBI + DPI + CO5300 and turn it on (ready for FillSolid on return).
    esp_err_t Init(const Co5300Config& cfg);

    // Fill the whole screen with one RGB565 color. Valid only after Init() succeeds.
    esp_err_t FillSolid(uint16_t rgb565_color);

    // Screen brightness 0-255 (DBI write 0x51). An AMOLED has no backlight panel; brightness is controlled inside the CO5300.
    esp_err_t SetBrightness(uint8_t level);

    esp_err_t DisplayOn();
    esp_err_t DisplayOff();

    uint16_t Width()  const { return cfg_.width; }
    uint16_t Height() const { return cfg_.height; }
    bool     Ready()  const { return panel_ != nullptr; }

private:
    Co5300Config cfg_       = {};
    void*        ldo_       = nullptr;  // esp_ldo_channel_handle_t
    void*        dsi_bus_   = nullptr;  // esp_lcd_dsi_bus_handle_t
    void*        io_        = nullptr;  // esp_lcd_panel_io_handle_t (DBI command port)
    void*        panel_     = nullptr;  // esp_lcd_panel_handle_t (CO5300, includes DPI)
    uint8_t*     fb_        = nullptr;  // full-screen framebuffer (PSRAM)
    size_t       fb_bytes_  = 0;
};
