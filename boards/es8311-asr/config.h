#ifndef _AGENT_LINK_BOARD_CONFIG_H_
#define _AGENT_LINK_BOARD_CONFIG_H_

#include <driver/gpio.h>

// ES8311 ASR : ESP32-S3 minimal real-time ASR example
// One ES8311 over a full-duplex standard-I2S port; this board only uses the mic (ADC) to stream audio to the App for live transcription
#define AUDIO_I2S_MCLK          GPIO_NUM_46   // MCK
#define AUDIO_I2S_BCLK          GPIO_NUM_39   // SCLK
#define AUDIO_I2S_WS            GPIO_NUM_2    // LR / LRCK
#define AUDIO_I2S_DOUT          GPIO_NUM_38   // DI : speaker (unused on this board)
#define AUDIO_I2S_DIN           GPIO_NUM_40   // DO : mic
#define AUDIO_PA_EN             GPIO_NUM_NC   // no external PA on this example board

// I2C:configures the ES8311 registers
#define AUDIO_I2C_SDA           GPIO_NUM_41
#define AUDIO_I2C_SCL           GPIO_NUM_42
#define ES8311_ADDR             0x18          // AD0 low; 0x19 if AD0 is tied high
#define AUDIO_SAMPLE_RATE       16000

// Push-to-talk button (active-HIGH: idles low, pressed drives high). GPIO8 is a normal (non-strapping) pin.
#define BUTTON_BOOT_PIN         GPIO_NUM_8

#endif  // _AGENT_LINK_BOARD_CONFIG_H_
