#ifndef _AGENT_LINK_BOARD_CONFIG_H_
#define _AGENT_LINK_BOARD_CONFIG_H_

#include <driver/gpio.h>

// ES8311 Voice — ESP32-S3 single-codec audio example
// One ES8311 does both speaker (DAC) and mic (ADC) over a full-duplex standard-I2S port
// the shared Es8311Codec driver (boards/common) takes these pins
#define AUDIO_I2S_MCLK          GPIO_NUM_46   // MCK
#define AUDIO_I2S_BCLK          GPIO_NUM_39   // SCLK
#define AUDIO_I2S_WS            GPIO_NUM_2    // LR / LRCK
#define AUDIO_I2S_DOUT          GPIO_NUM_38   // DI : speaker
#define AUDIO_I2S_DIN           GPIO_NUM_40   // DO : mic
#define AUDIO_PA_EN             GPIO_NUM_NC   // no external PA on this example board

// I2C — configures the ES8311 registers
#define AUDIO_I2C_SDA           GPIO_NUM_41
#define AUDIO_I2C_SCL           GPIO_NUM_42
#define ES8311_ADDR             0x18          // AD0 low; 0x19 if AD0 is tied high
#define AUDIO_SAMPLE_RATE       16000

// Push-to-talk button (BOOT key): hold to speak, release to end
#define BUTTON_BOOT_PIN         GPIO_NUM_8

#endif  // _AGENT_LINK_BOARD_CONFIG_H_
