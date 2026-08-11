#ifndef _AGENT_LINK_BOARD_CONFIG_H_
#define _AGENT_LINK_BOARD_CONFIG_H_

#include <driver/gpio.h>
#include <driver/i2c_master.h>   // i2c_port_t / I2C_NUM_0
#include <driver/spi_common.h>   // spi_host_device_t / SPI2_HOST

// ── I2C: SPA06 (pressure) and SHT30 (humidity) share one bus ──
#define I2C_PORT      I2C_NUM_0
#define I2C_SDA       GPIO_NUM_39
#define I2C_SCL       GPIO_NUM_40
#define I2C_FREQ_HZ   400000

// SPA06 pressure/temperature sensor: SDO to VDD = 0x77, to GND = 0x76
#define SPA06_ADDR    0x77

// SHT30 temperature/humidity sensor: ADDR to GND = 0x44, to VDD = 0x45
#define SHT30_ADDR    0x44

// ── ST7789 display (SPI, 240x240) — set these to your actual wiring ──
#define DISPLAY_SPI_HOST    SPI2_HOST
#define DISPLAY_PIN_SCK     GPIO_NUM_12
#define DISPLAY_PIN_MOSI    GPIO_NUM_11   // a.k.a. SDA on ST7789 modules
#define DISPLAY_PIN_CS      GPIO_NUM_10   // GPIO_NUM_NC if CS is tied to GND
#define DISPLAY_PIN_DC      GPIO_NUM_9    // a.k.a. RS
#define DISPLAY_PIN_RST     GPIO_NUM_8
#define DISPLAY_PIN_BL      GPIO_NUM_7    // backlight; GPIO_NUM_NC if always on
#define DISPLAY_WIDTH       240
#define DISPLAY_HEIGHT      240
#define DISPLAY_SPI_CLK_HZ  (40 * 1000 * 1000)

#endif  // _AGENT_LINK_BOARD_CONFIG_H_
