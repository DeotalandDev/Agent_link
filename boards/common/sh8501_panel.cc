#include "sh8501_panel.h"

#include <algorithm>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_sh8501.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
constexpr const char* TAG = "sh8501";
// Stripe height: 120w x 20 x 2B = 4800B, fits internal DMA SRAM and still fills the screen in a few passes.
constexpr uint16_t kStripeRows = 20;

// panel-io done callback (C signature) -> forward to the instance.
bool ColorTransDone(esp_lcd_panel_io_handle_t /*io*/,
                    esp_lcd_panel_io_event_data_t* /*edata*/,
                    void* user_ctx) {
    auto* self = static_cast<Sh8501Panel*>(user_ctx);
    return self ? self->NotifyDoneFromIsr() : false;
}
}  // namespace

Sh8501Panel::~Sh8501Panel() {
    if (stripe_) { heap_caps_free(stripe_); stripe_ = nullptr; }
    if (panel_)  { esp_lcd_panel_del(panel_); panel_ = nullptr; }
    if (io_)     { esp_lcd_panel_io_del(io_); io_ = nullptr; }
    if (tx_sem_) { vSemaphoreDelete(static_cast<SemaphoreHandle_t>(tx_sem_)); tx_sem_ = nullptr; }
}

bool Sh8501Panel::NotifyDoneFromIsr() {
    done_count_.fetch_add(1, std::memory_order_release);
    auto sem = static_cast<SemaphoreHandle_t>(tx_sem_);
    if (!sem) return false;
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(sem, &hp);
    return hp == pdTRUE;
}

bool Sh8501Panel::WaitDone(uint32_t target, uint32_t timeout_ms) {
    auto sem = static_cast<SemaphoreHandle_t>(tx_sem_);
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (done_count_.load(std::memory_order_acquire) < target) {
        const TickType_t now = xTaskGetTickCount();
        if (now >= deadline) return false;
        if (sem) (void)xSemaphoreTake(sem, deadline - now);
        else     vTaskDelay(1);
    }
    return true;
}

esp_err_t Sh8501Panel::Init(const Sh8501Config& cfg) {
    cfg_ = cfg;

    tx_sem_ = xSemaphoreCreateBinary();
    if (!tx_sem_) return ESP_ERR_NO_MEM;

    // Defensive: clear any RST/CS pad hold left from a previous run (light sleep / abnormal reset), or the reset pulse won't come out.
    (void)gpio_hold_dis(static_cast<gpio_num_t>(cfg_.pin_rst));
    (void)gpio_hold_dis(static_cast<gpio_num_t>(cfg_.pin_cs));

    // ── SPI bus ──
    spi_bus_config_t bus = {};
    bus.sclk_io_num     = cfg_.pin_sck;
    bus.mosi_io_num     = cfg_.pin_mosi;
    bus.miso_io_num     = -1;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = static_cast<int>(cfg_.width) * kStripeRows * 2;
    esp_err_t ret = spi_bus_initialize(cfg_.spi_host, &bus, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // INVALID_STATE = already initialized, tolerate
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(ret));
        return ret;
    }

    // ── panel-io (SPI) ──
    esp_lcd_panel_io_spi_config_t io_cfg = {};
    io_cfg.dc_gpio_num       = cfg_.pin_dc;
    io_cfg.cs_gpio_num       = cfg_.pin_cs;
    io_cfg.pclk_hz           = cfg_.pclk_hz;
    io_cfg.lcd_cmd_bits      = 32;               // SH8501: 32-bit command header
    io_cfg.lcd_param_bits    = 8;
    io_cfg.spi_mode          = cfg_.spi_mode & 3;
    io_cfg.trans_queue_depth = 1;                // long-stable value on SH8501
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi(
            static_cast<esp_lcd_spi_bus_handle_t>(cfg_.spi_host), &io_cfg, &io_),
        TAG, "new_panel_io_spi");

    esp_lcd_panel_io_callbacks_t cbs = {};
    cbs.on_color_trans_done = ColorTransDone;
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_register_event_callbacks(io_, &cbs, this),
        TAG, "register cb");

    // ── panel (SH8501): init_cmds=NULL -> the component's built-in full sequence (doesn't rely on panel OTP, so sample panels light up too) ──
    sh8501_vendor_config_t vendor = {};
    vendor.flags.use_qspi_interface = 0;

    esp_lcd_panel_dev_config_t panel_cfg = {};
    panel_cfg.reset_gpio_num = cfg_.pin_rst;
    panel_cfg.rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_cfg.bits_per_pixel = 16;
    panel_cfg.vendor_config  = &vendor;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_sh8501(io_, &panel_cfg, &panel_), TAG, "new_panel");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel_), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel_),  TAG, "init");

    // Init tail: TE on / display control / idle off (ported from a verified sequence).
    const uint8_t te_on[]     = { 0x00 };
    const uint8_t disp_ctrl[] = { 0x20 };
    (void)esp_lcd_panel_io_tx_param(io_, 0x35, te_on, sizeof(te_on));
    (void)esp_lcd_panel_io_tx_param(io_, 0x53, disp_ctrl, sizeof(disp_ctrl));
    (void)esp_lcd_panel_io_tx_param(io_, 0x39, nullptr, 0);

    // Suppress the boot color noise: DISPOFF hides the clear -> clear GRAM to black -> DISPON (lit black) -> raise brightness.
    (void)esp_lcd_panel_disp_on_off(panel_, false);
    (void)FillSolid(rgb565::kBlack);
    (void)esp_lcd_panel_disp_on_off(panel_, true);
    (void)SetBrightness(0xFF);

    ESP_LOGI(TAG, "SH8501 panel ready (%ux%u @ %luMHz mode%u)",
             cfg_.width, cfg_.height,
             static_cast<unsigned long>(cfg_.pclk_hz / 1000000), cfg_.spi_mode & 3);
    return ESP_OK;
}

esp_err_t Sh8501Panel::FillSolid(uint16_t color) {
    if (!panel_) return ESP_ERR_INVALID_STATE;

    const size_t stripe_bytes = static_cast<size_t>(cfg_.width) * kStripeRows * 2u;
    if (!stripe_ || stripe_cap_ < stripe_bytes) {
        if (stripe_) heap_caps_free(stripe_);
        // Must be internal DMA-capable SRAM: the esp_lcd SPI backend uses it directly as the DMA source.
        stripe_ = static_cast<uint8_t*>(
            heap_caps_aligned_alloc(4, stripe_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (!stripe_) { stripe_cap_ = 0; return ESP_ERR_NO_MEM; }
        stripe_cap_ = stripe_bytes;
    }

    // RGB565 big-endian fill: [hi, lo, hi, lo, ...]
    const uint8_t hi = static_cast<uint8_t>(color >> 8);
    const uint8_t lo = static_cast<uint8_t>(color & 0xFF);
    for (size_t i = 0; i < stripe_bytes; i += 2u) { stripe_[i] = hi; stripe_[i + 1u] = lo; }

    const uint16_t w = cfg_.width;
    const uint16_t h = cfg_.height;
    for (uint16_t row = 0; row < h; row = static_cast<uint16_t>(row + kStripeRows)) {
        const uint16_t rows_this = std::min<uint16_t>(kStripeRows, static_cast<uint16_t>(h - row));
        const uint32_t expect = done_count_.load(std::memory_order_acquire) + 1;
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_draw_bitmap(panel_, 0, row, w, static_cast<uint16_t>(row + rows_this), stripe_),
            TAG, "draw");
        if (!WaitDone(expect, 500)) {   // single buffer: the next stripe reuses the same buf, so wait for this DMA to finish
            ESP_LOGE(TAG, "fill tx timeout row=%u", static_cast<unsigned>(row));
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

esp_err_t Sh8501Panel::SetBrightness(uint8_t level) {
    if (!io_) return ESP_ERR_INVALID_STATE;
    const uint8_t p[] = { level };
    return esp_lcd_panel_io_tx_param(io_, 0x51, p, sizeof(p));
}

esp_err_t Sh8501Panel::DisplayOn()  {
    return panel_ ? esp_lcd_panel_disp_on_off(panel_, true)  : ESP_ERR_INVALID_STATE;
}
esp_err_t Sh8501Panel::DisplayOff() {
    return panel_ ? esp_lcd_panel_disp_on_off(panel_, false) : ESP_ERR_INVALID_STATE;
}
