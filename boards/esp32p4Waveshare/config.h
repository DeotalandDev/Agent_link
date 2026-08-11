#ifndef _AGENT_LINK_BOARD_CONFIG_H_
#define _AGENT_LINK_BOARD_CONFIG_H_

#include <driver/gpio.h>

#define DISPLAY_WIDTH    466
#define DISPLAY_HEIGHT   466

#define DISPLAY_RST_PIN     GPIO_NUM_47
#define DISPLAY_PWR_EN_PIN  GPIO_NUM_46

#define TOUCH_I2C_SCL    GPIO_NUM_8
#define TOUCH_I2C_SDA    GPIO_NUM_7

// #define CST820_ADDR   0x15

#endif  // _AGENT_LINK_BOARD_CONFIG_H_
