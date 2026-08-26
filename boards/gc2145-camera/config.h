#ifndef _AGENT_LINK_BOARD_CONFIG_H_
#define _AGENT_LINK_BOARD_CONFIG_H_

#include <driver/gpio.h>

// GC2145 Camera: live DVP camera preview on an ST7789 240x240 SPI LCD

// Camera: DVP 8-bit + SCCB(I2C) + XCLK. -1 = not wired
#define CAM_PIN_PWDN            -1              // power-down
#define CAM_PIN_RESET           -1              // reset
#define CAM_PIN_XCLK            GPIO_NUM_15     // master clock out to sensor
#define CAM_PIN_SIOD            GPIO_NUM_4      // SCCB SDA
#define CAM_PIN_SIOC            GPIO_NUM_5      // SCCB SCL
#define CAM_PIN_D7              GPIO_NUM_16     // Y9
#define CAM_PIN_D6              GPIO_NUM_17     // Y8
#define CAM_PIN_D5              GPIO_NUM_18     // Y7
#define CAM_PIN_D4              GPIO_NUM_12     // Y6
#define CAM_PIN_D3              GPIO_NUM_10     // Y5
#define CAM_PIN_D2              GPIO_NUM_8      // Y4
#define CAM_PIN_D1              GPIO_NUM_9      // Y3
#define CAM_PIN_D0              GPIO_NUM_11     // Y2
#define CAM_PIN_VSYNC           GPIO_NUM_6
#define CAM_PIN_HREF            GPIO_NUM_7
#define CAM_PIN_PCLK            GPIO_NUM_13

#define CAM_XCLK_FREQ_HZ        (20 * 1000 * 1000)

// Snapshot trigger button — idles low (internal pull-down), driven high when pressed
#define CAPTURE_BUTTON_PIN      GPIO_NUM_3

// Onboard WS2812 (NeoPixel) RGB LED on GPIO48, driven over RMT. Exposed to the App as the "led0" endpoint.
#define WS2812_LED_PIN          GPIO_NUM_48

#define CAMERA_MASK_TOP_ROWS 8

// set this to 1 to byte-swap each pixel before drawing
#define CAMERA_RGB565_BYTE_SWAP 0

// Display: ST7789 240x240
#define DISPLAY_SPI_HOST        SPI2_HOST
#define DISPLAY_SCK_PIN         GPIO_NUM_21     // SCL
#define DISPLAY_MOSI_PIN        GPIO_NUM_47     // SDA
#define DISPLAY_DC_PIN          GPIO_NUM_43     //
#define DISPLAY_CS_PIN          GPIO_NUM_44     //
#define DISPLAY_RST_PIN         (-1)
#define DISPLAY_BL_PIN          (-1)
#define DISPLAY_WIDTH           240
#define DISPLAY_HEIGHT          240

#define DISPLAY_SPI_CLK_HZ      (40 * 1000 * 1000)
#define DISPLAY_SPI_MODE        0

#endif  // _AGENT_LINK_BOARD_CONFIG_H_
