# boards/common/ — cross-board drivers

Drivers and helpers that more than one board may use (chip drivers, bus helpers). The board only passes
pins; the driver itself is board-independent. `main/CMakeLists.txt` compiles every `*.cc`/`*.c` here, and
the include path `../boards/common` lets a board `#include "xxx.h"` directly.

| Files                                      | Purpose                                                                                     | Dependencies (in main's REQUIRES)                                |
| ------------------------------------------ | ------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| `sh8501_panel.{h,cc}`                      | SH8501 AMOLED (SPI): init + solid fill + brightness; pins come from `Sh8501Config`          | `esp_lcd`, `esp_lcd_sh8501`, `esp_driver_spi`, `esp_driver_gpio` |
| `es_codec.{h,cc}`                          | ES8311 + ES7210 full-duplex audio codec (speaker out / mic in)                              | `esp_codec_dev`, `esp_driver_i2c`, `esp_driver_i2s`              |
| `es8311_audio.{h,cc}`                      | ES8311-only full-duplex codec: one chip does both speaker (DAC) and mic (ADC), standard I2S | `esp_codec_dev`, `esp_driver_i2c`, `esp_driver_i2s`              |
| `co5300_panel.{h,cc}` + `co5300_hal.{c,h}` | CO5300 466x466 AMOLED (MIPI-DSI); ESP32-P4 only (stubbed out on other targets)              | `esp_lcd`, `esp_lcd_co5300`                                      |
| `bq27220.{h,cc}`                           | BQ27220 fuel gauge (I2C); reuses an I2C bus another driver already created                  | `esp_driver_i2c`                                                 |

## Usage (from a board)

```cpp
#include "sh8501_panel.h"

Sh8501Config c = {};
c.spi_host = DISPLAY_SPI_HOST;  c.pin_sck = DISPLAY_SCK_PIN;  c.pin_mosi = DISPLAY_MOSI_PIN;
c.pin_cs = DISPLAY_CS_PIN;  c.pin_dc = DISPLAY_DC_PIN;  c.pin_rst = DISPLAY_RST_PIN;
c.width = DISPLAY_WIDTH;  c.height = DISPLAY_HEIGHT;  c.pclk_hz = DISPLAY_SPI_CLK_HZ;  c.spi_mode = DISPLAY_SPI_MODE;

Sh8501Panel panel;
panel.Init(c);                    // init + light up (ends black, full brightness)
panel.FillSolid(rgb565::kBlue);   // fill the whole screen
```

## Rules for adding a shared driver

- **Board-independent**: pass pin/address differences in through a parameter/Config; don't hardcode one board's pins here.
- **Only put things that are genuinely reused.** A private driver used by a single board belongs in that board's own directory (`boards/<board>/`).
- If a new driver needs a new ESP-IDF component, add it to `REQUIRES` in `main/CMakeLists.txt`.

## Note: the SH8501 driver ships with the repo

The low-level panel driver `esp_lcd_sh8501` that `sh8501_panel` depends on isn't in the Espressif component
registry, so it's vendored in `components/esp_lcd_sh8501/` (Espressif's reference implementation plus this
project's brightness patch). That keeps the project self-contained and buildable offline.
