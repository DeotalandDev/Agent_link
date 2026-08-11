// Co5300Panel: CO5300 466x466 round AMOLED (MIPI-DSI) reusable driver implementation, see co5300_panel.h.
//
// Split of responsibilities:
//   - Low-level bring-up (LDO/DSI/DBI/DPI/CO5300, using esp_lcd_co5300's C-only config macros) -> co5300_hal.c (compiled as C).
//   - This C++ file only wraps it for boards: it calls co5300_hal_init to bring up the chain, then uses the
//     generic esp_lcd API for fill/brightness. Those generic APIs (draw_bitmap / io_tx_param / disp_on_off) are
//     plain functions usable from C++ and common across chips, so this file has no MIPI-only macros and needs no
//     per-target gating (on non-P4, hal returns "not supported" and panel_ stays null, which no-ops safely).
#include "co5300_panel.h"
#include "co5300_hal.h"

#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "co5300";
}  // namespace

Co5300Panel::~Co5300Panel() {
    if (fb_)    { heap_caps_free(fb_); fb_ = nullptr; }
    if (panel_) { esp_lcd_panel_del(static_cast<esp_lcd_panel_handle_t>(panel_)); panel_ = nullptr; }
    if (io_)    { esp_lcd_panel_io_del(static_cast<esp_lcd_panel_io_handle_t>(io_)); io_ = nullptr; }
    // Freeing dsi_bus_/ldo_ needs MIPI-only headers; this board is a resident singleton whose destructor almost never runs, so we skip deleting them one by one.
}

esp_err_t Co5300Panel::Init(const Co5300Config& cfg) {
    cfg_ = cfg;

    // Warn loudly when the reset pin is unset (placeholder -1): the CO5300 datasheet requires an active-low hardware reset to initialize correctly.
    if (cfg_.rst_gpio < 0) {
        ESP_LOGW(TAG, "reset pin not configured (placeholder %d): the CO5300 needs an active-low "
                      "hardware reset, so the panel may not initialize; set DISPLAY_RST_PIN to a real "
                      "GPIO in config.h and reflash", cfg_.rst_gpio);
    }

    // Hand the low-level bring-up to the C side (isolates esp_lcd_co5300's C-only macros, see co5300_hal.c).
    co5300_hal_handles_t h = {};
    const esp_err_t err = co5300_hal_init(cfg_.rst_gpio, cfg_.pwr_en_gpio, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "co5300_hal_init failed: %s", esp_err_to_name(err));
        return err;
    }
    ldo_ = h.ldo; dsi_bus_ = h.dsi_bus; io_ = h.io; panel_ = h.panel;

    (void)SetBrightness(0xFF);   // AMOLED brightness goes through DBI 0x51; set it to max so the self-test is visible
    ESP_LOGI(TAG, "CO5300 %ux%u MIPI-DSI ready (rst=%d)", cfg_.width, cfg_.height, cfg_.rst_gpio);
    return ESP_OK;
}

esp_err_t Co5300Panel::FillSolid(uint16_t color) {
    if (!panel_) return ESP_ERR_INVALID_STATE;
    const size_t px    = static_cast<size_t>(cfg_.width) * cfg_.height;
    const size_t bytes = px * sizeof(uint16_t);            // 466*466*2 ~= 434KB
    if (!fb_ || fb_bytes_ < bytes) {
        if (fb_) heap_caps_free(fb_);
        // The full-screen framebuffer must live in PSRAM (434KB will not fit in internal SRAM) -> config.json needs CONFIG_SPIRAM=y.
        fb_ = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM));
        if (!fb_) { fb_bytes_ = 0; return ESP_ERR_NO_MEM; }
        fb_bytes_ = bytes;
    }
    // The MIPI DPI RGB565 framebuffer is native-endian uint16 (unlike SH8501's hand-packed big-endian).
    // If self-test colors are wrong (e.g. red shows as blue), it is usually a byte-order / element-order issue; try rgb_ele_order first.
    uint16_t* p = reinterpret_cast<uint16_t*>(fb_);
    for (size_t i = 0; i < px; ++i) p[i] = color;
    return esp_lcd_panel_draw_bitmap(static_cast<esp_lcd_panel_handle_t>(panel_),
                                     0, 0, cfg_.width, cfg_.height, fb_);
}

esp_err_t Co5300Panel::SetBrightness(uint8_t level) {
    if (!io_) return ESP_ERR_INVALID_STATE;
    const uint8_t p[] = { level };
    return esp_lcd_panel_io_tx_param(static_cast<esp_lcd_panel_io_handle_t>(io_), 0x51, p, sizeof(p));
}

esp_err_t Co5300Panel::DisplayOn() {
    return panel_ ? esp_lcd_panel_disp_on_off(static_cast<esp_lcd_panel_handle_t>(panel_), true)
                  : ESP_ERR_INVALID_STATE;
}
esp_err_t Co5300Panel::DisplayOff() {
    return panel_ ? esp_lcd_panel_disp_on_off(static_cast<esp_lcd_panel_handle_t>(panel_), false)
                  : ESP_ERR_INVALID_STATE;
}
