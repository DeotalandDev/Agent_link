// CO5300 (MIPI-DSI) low-level bring-up, C implementation (pairs with the C++ class in co5300_panel.cc). See co5300_hal.h.

// ESP32-P4, so it is gated on CONFIG_IDF_TARGET_ESP32P4: on non-P4 it compiles to an empty stub so
#include "co5300_hal.h"

#if CONFIG_IDF_TARGET_ESP32P4
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_co5300.h"
#include "esp_log.h"

#define CO5300_LDO_CHAN 3
#define CO5300_LDO_MV   2500

static const char *TAG = "co5300_hal";

static const co5300_lcd_init_cmd_t s_co5300_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},                     // command-page select (user page)
    {0xC4, (uint8_t[]){0x80}, 1, 0},                     // interface setting (vendor note: SPI setting, mipi remove)
    {0x3A, (uint8_t[]){0x55}, 1, 0},                     // pixel format 0x55 = RGB565 (0x77 = RGB888)
    {0x35, (uint8_t[]){0x00}, 1, 0},                     // Tearing Effect line on
    {0x53, (uint8_t[]){0x20}, 1, 0},                     // brightness control enable
    {0x51, (uint8_t[]){0xFF}, 1, 0},                     // display brightness = max
    {0x63, (uint8_t[]){0xFF}, 1, 0},                     // HBM brightness
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0},   // column address 6..471 (column offset 6)
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 0},   // row address 0..465
    {0x11, NULL, 0, 60},                                 // sleep out, wait 60ms
    {0x29, NULL, 0, 0},                                  // display on
};

esp_err_t co5300_hal_init(int rst_gpio, int pwr_en_gpio, co5300_hal_handles_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Power the panel first
    if (pwr_en_gpio >= 0) {
        gpio_config_t pwr = {
            .pin_bit_mask = 1ULL << pwr_en_gpio,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&pwr);
        gpio_set_level(pwr_en_gpio, 1);      // drive high = power the panel
        vTaskDelay(pdMS_TO_TICKS(50));       // let the supply settle before reset/init
    }

    // (1) On-chip LDO supplies 2.5V to the MIPI D-PHY (must be before creating the DSI bus).
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = CO5300_LDO_CHAN,
        .voltage_mv = CO5300_LDO_MV,
    };
    esp_ldo_channel_handle_t ldo = NULL;
    esp_err_t err = esp_ldo_acquire_channel(&ldo_cfg, &ldo);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ldo acquire: %s", esp_err_to_name(err));
        return err;
    }

    // (2) DSI bus (lane count / bitrate set by the component macro for this panel; 1CH = one data lane, on D0).
    //     Your board wires both D0/D1; if this panel needs 2 lanes, swap the 1CH macro below for 2CH.
    esp_lcd_dsi_bus_config_t bus_cfg = CO5300_PANEL_BUS_DSI_1CH_CONFIG();
    esp_lcd_dsi_bus_handle_t dsi = NULL;
    err = esp_lcd_new_dsi_bus(&bus_cfg, &dsi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "new dsi bus: %s", esp_err_to_name(err));
        return err;
    }

    // (3) DBI command channel (sends the vendor init sequence / brightness 0x51).
    esp_lcd_dbi_io_config_t dbi_cfg = CO5300_PANEL_IO_DBI_CONFIG();
    esp_lcd_panel_io_handle_t io = NULL;
    err = esp_lcd_new_panel_io_dbi(dsi, &dbi_cfg, &io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "new dbi io: %s", esp_err_to_name(err));
        return err;
    }

    // (4) DPI video stream (466x466 @ 60Hz, RGB565). Note: ESP-IDF >= 6.0 switches to ..._DPI_CONFIG_CF().
    esp_lcd_dpi_panel_config_t dpi_cfg =
        CO5300_466_466_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);

    // (5) CO5300 panel: bundles DBI (commands) + DPI (pixels); it sends the vendor sequence over DBI internally.
    co5300_vendor_config_t vendor = {
        .init_cmds = s_co5300_init_cmds,                                    // use the vendor sequence (leave unset for the component default)
        .init_cmds_size = sizeof(s_co5300_init_cmds) / sizeof(s_co5300_init_cmds[0]),
        .flags.use_mipi_interface = 1,
        .mipi_config = {
            .dsi_bus = dsi,
            .dpi_config = &dpi_cfg,
        },
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = rst_gpio,                   // active-low is guaranteed by the default flags.reset_active_high=0
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,   // if colors are swapped (RGB<->BGR) change this
        .bits_per_pixel = 16,                         // RGB565
        .vendor_config = &vendor,
    };
    esp_lcd_panel_handle_t panel = NULL;
    err = esp_lcd_new_panel_co5300(io, &panel_cfg, &panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "new panel co5300: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_lcd_panel_reset(panel);   // pulse the active-low reset (no-op when rst<0 placeholder)
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel reset: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_lcd_panel_init(panel);    // run the vendor sequence (includes 0x11 sleep-out / 0x29 display-on)
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel init: %s", esp_err_to_name(err));
        return err;
    }
    // Don't call esp_lcd_panel_disp_on_off: MIPI DPI panels don't support that op (it logs an error);
    // display on/off is handled by 0x29 in the init sequence

    out->ldo = ldo;
    out->dsi_bus = dsi;
    out->io = io;
    out->panel = panel;
    return ESP_OK;
}

#else  // ── Non-ESP32-P4: empty stub (only so boards/common GLOB compiles across boards) ──
esp_err_t co5300_hal_init(int rst_gpio, int pwr_en_gpio, co5300_hal_handles_t *out)
{
    (void)rst_gpio;
    (void)pwr_en_gpio;
    (void)out;
    return ESP_ERR_NOT_SUPPORTED;
}
#endif
