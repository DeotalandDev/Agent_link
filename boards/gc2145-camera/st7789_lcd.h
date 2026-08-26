#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_types.h"

namespace rgb565 {
constexpr uint16_t kBlack = 0x0000;
constexpr uint16_t kWhite = 0xFFFF;
constexpr uint16_t kRed   = 0xF800;
constexpr uint16_t kGreen = 0x07E0;
constexpr uint16_t kBlue  = 0x001F;
}  // namespace rgb565

struct St7789LcdConfig {
    spi_host_device_t spi_host;
    int      pin_sck;
    int      pin_mosi;
    int      pin_cs;                    // -1 if CS is tied low
    int      pin_dc;
    int      pin_rst;                   // -1 if not wired (software reset)
    int      pin_bl         = -1;       // backlight GPIO; -1 = none / always on
    uint16_t width          = 240;
    uint16_t height         = 240;
    uint32_t pclk_hz        = 40 * 1000 * 1000;
    uint8_t  spi_mode       = 0;        // ST7789 is SPI mode 0
    bool     invert_color   = true;
    bool     bgr            = false;    // set true if red and blue look swapped
    bool     mirror_x       = false;
    bool     mirror_y       = false;
    bool     swap_xy        = false;
    int      gap_x          = 0;
    int      gap_y          = 0;
    bool     bl_active_high = true;
};

class St7789Lcd {
public:
    St7789Lcd() = default;
    ~St7789Lcd();
    St7789Lcd(const St7789Lcd&) = delete;
    St7789Lcd& operator=(const St7789Lcd&) = delete;

    // Bring up SPI + panel and turn the display on (cleared to black, backlight on).
    esp_err_t Init(const St7789LcdConfig& cfg);

    // Fill the whole screen with one RGB565 color.
    esp_err_t FillSolid(uint16_t rgb565_color);

    // Blit a w*h RGB565 frame at (x,y). Blocks until the DMA completes, so 'pixels' may be reused/freed immediately after
    // 'pixels' may live in PSRAM. Byte order must match the panel
    esp_err_t DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const void* pixels);

    esp_err_t Backlight(bool on);
    esp_err_t DisplayOn();
    esp_err_t DisplayOff();

    uint16_t Width()  const { return cfg_.width; }
    uint16_t Height() const { return cfg_.height; }
    bool     Ready()  const { return panel_ != nullptr; }

    // Called by the panel-io "transfer done" callback; do not call directly
    bool NotifyDoneFromIsr();

private:
    bool WaitDone(uint32_t target, uint32_t timeout_ms);
    esp_err_t EnsureStripe(size_t bytes);

    St7789LcdConfig           cfg_        = {};
    esp_lcd_panel_io_handle_t io_         = nullptr;
    esp_lcd_panel_handle_t    panel_      = nullptr;
    void*                     tx_sem_     = nullptr;   // SemaphoreHandle_t
    uint8_t*                  stripe_[2]  = {nullptr, nullptr};  // ping-pong internal DMA SRAM (see DrawBitmap)
    size_t                    stripe_cap_ = 0;         // capacity of EACH stripe buffer
    std::atomic<uint32_t>     done_count_{0};          // ISR increments; the task waits for it to catch up
};
