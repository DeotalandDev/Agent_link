#pragma once
// C interface for the CO5300 (MIPI-DSI) low-level bring-up, called from the C++ Co5300Panel

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Handles returned after bring-up
typedef struct {
    void *ldo;      // esp_ldo_channel_handle_t
    void *dsi_bus;  // esp_lcd_dsi_bus_handle_t
    void *io;       // esp_lcd_panel_io_handle_t (DBI command port)
    void *panel;    // esp_lcd_panel_handle_t (CO5300, includes DPI)
} co5300_hal_handles_t;

// Bring up the full LDO -> DSI -> DBI -> DPI -> CO5300 chain and turn the panel on. rst_gpio < 0 means no reset pin (placeholder).
// pwr_en_gpio: panel power-enable pin (active high, e.g. VCI_EN), < 0 = none; driven high with a settle delay before reset/init.
// Returns ESP_OK and fills out on success; returns an error code on failure (out contents undefined).
esp_err_t co5300_hal_init(int rst_gpio, int pwr_en_gpio, co5300_hal_handles_t *out);

#ifdef __cplusplus
}
#endif
