#pragma once
// Sh8501Panel: reusable driver for the SH8501 AMOLED (SPI), in boards/common.
//
// The board supplies pins + resolution; this class does the SPI bus -> panel-io -> panel init
// (including the vendor sequence) -> clear -> light up -> solid fill / brightness. Board-independent
// any board with an SH8501 SPI panel can reuse it, passing pins through Config
//
// Note: this is a reference implementation that only does light-up + solid-color fill + brightness.
// A real product doing animation has concurrent blits and would need ping-pong double buffering plus a
// panel-io serialization lock; this reference does not include that
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_types.h"

// Common RGB565 colors (big-endian is handled inside FillSolid).
namespace rgb565 {
constexpr uint16_t kBlack = 0x0000;
constexpr uint16_t kWhite = 0xFFFF;
constexpr uint16_t kRed   = 0xF800;
constexpr uint16_t kGreen = 0x07E0;
constexpr uint16_t kBlue  = 0x001F;
}  // namespace rgb565

struct Sh8501Config {
    spi_host_device_t spi_host;
    int      pin_sck;
    int      pin_mosi;
    int      pin_cs;
    int      pin_dc;
    int      pin_rst;
    uint16_t width;
    uint16_t height;
    uint32_t pclk_hz;
    uint8_t  spi_mode;   // SH8501 is usually mode 3
};

class Sh8501Panel {
public:
    Sh8501Panel() = default;
    ~Sh8501Panel();

    Sh8501Panel(const Sh8501Panel&) = delete;
    Sh8501Panel& operator=(const Sh8501Panel&) = delete;

    // Bring up SPI + panel and light it up (screen black, brightness maxed on return; ready for FillSolid).
    esp_err_t Init(const Sh8501Config& cfg);

    // Fill the whole screen with one RGB565 color. Valid only after Init() succeeds.
    esp_err_t FillSolid(uint16_t rgb565_color);

    // Screen brightness 0-255 (write 0x51).
    esp_err_t SetBrightness(uint8_t level);

    esp_err_t DisplayOn();
    esp_err_t DisplayOff();

    uint16_t Width()  const { return cfg_.width; }
    uint16_t Height() const { return cfg_.height; }
    bool     Ready()  const { return panel_ != nullptr; }

public:
    // Called by the panel-io callback when a stripe's SPI DMA finishes (used by the forwarder in the .cc; don't call directly).
    bool NotifyDoneFromIsr();

private:
    bool WaitDone(uint32_t target, uint32_t timeout_ms);

    Sh8501Config              cfg_        = {};
    esp_lcd_panel_io_handle_t io_         = nullptr;
    esp_lcd_panel_handle_t    panel_      = nullptr;
    void*                     tx_sem_     = nullptr;   // SemaphoreHandle_t
    uint8_t*                  stripe_     = nullptr;   // internal DMA SRAM
    size_t                    stripe_cap_ = 0;
    std::atomic<uint32_t>     done_count_{0};          // ISR increments; the task waits for it to reach the target
};
