// ES8311 Voice (ESP32-S3) — minimal audio example board.
//one ES8311 chip does full-duplex audio, giving the two voice paths and nothing else (no screen, no battery):
//    Speaker: Agent TTS arrives via PlayAudio(), is buffered, and a task drains it to the codec.
//    Mic:     holding the BOOT key streams push-to-talk audio up to the Agent.
// Copy this board as the starting point for any single-ES8311 device.

#include "board.h"
#include "config.h"
#include "es8311_audio.h"

#include "agent_link.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#define TAG "Es8311Voice"

class Es8311VoiceBoard : public Board {
public:
    Es8311VoiceBoard() {
        InitCodec();
        InitButton();
    }

    const char* Name() const override { return "ROROLEE_ES8311"; }

    uint32_t Capabilities() const override {
        return AGENT_CAP_MIC | AGENT_CAP_SPEAKER | AGENT_CAP_BUTTON | AGENT_CAP_RECORDING;
    }

    // Agent downlink TTS (PCM16 / 16k / mono): push into the play buffer and return immediately without
    // blocking (this callback runs in the transport/host task context and must never block). PlayLoop
    // drains it to the codec. If the buffer is full (link burst > realtime), drop and warn.
    void PlayAudio(const uint8_t* pcm16, size_t bytes) override {
        if (!play_buf_ || !pcm16 || bytes == 0) return;
        const size_t sent = xStreamBufferSend(play_buf_, pcm16, bytes, 0);  // timeout 0 = non-blocking
        if (sent < bytes) {
            ESP_LOGW(TAG, "play buffer full — dropped %u/%u bytes", (unsigned)(bytes - sent), (unsigned)bytes);
        }
    }
    // End of a TTS segment: PlayLoop keeps playing whatever is left in the buffer (not flushed).
    void AudioEnd() override { ESP_LOGD(TAG, "AudioEnd (end of TTS segment; buffer plays out then goes silent)"); }

private:
    // ── Audio codec (ES8311) ──
    void InitCodec() {
        Es8311Config c = {};
        c.i2c_port    = I2C_NUM_0;
        c.pin_sda     = AUDIO_I2C_SDA;
        c.pin_scl     = AUDIO_I2C_SCL;
        c.pin_mclk    = AUDIO_I2S_MCLK;
        c.pin_bclk    = AUDIO_I2S_BCLK;
        c.pin_ws      = AUDIO_I2S_WS;
        c.pin_din     = AUDIO_I2S_DIN;    // mic
        c.pin_dout    = AUDIO_I2S_DOUT;   // speaker
        c.pin_pa_en   = AUDIO_PA_EN;
        c.es8311_addr = ES8311_ADDR;
        c.sample_rate = AUDIO_SAMPLE_RATE;
        c.mic_gain    = 30;
        c.out_volume  = 80;
        if (codec_.Init(c) != ESP_OK) { ESP_LOGE(TAG, "codec init failed"); return; }
        codec_ok_ = true;

        play_buf_ = xStreamBufferCreate(kPlayBufBytes, /*trigger=*/1);
        if (!play_buf_) { ESP_LOGE(TAG, "play buffer alloc failed"); return; }
        xTaskCreate(&Es8311VoiceBoard::PlayTaskEntry, "spk_play", 4096, this, 5, &play_task_);
    }

    static void PlayTaskEntry(void* arg) { static_cast<Es8311VoiceBoard*>(arg)->PlayLoop(); }

    // Pull PCM from the play buffer and write it to the codec.
    void PlayLoop() {
        int16_t buf[256];
        while (true) {
            const size_t n = xStreamBufferReceive(play_buf_, buf, sizeof(buf), portMAX_DELAY);
            if (n >= sizeof(int16_t)) codec_.WritePcm(buf, n / sizeof(int16_t));
        }
    }

    // ── Push-to-talk button ──
    // GPIO0 (BOOT) grounds when pressed, so it idles high with the internal pull-up.
    void InitButton() {
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << BUTTON_BOOT_PIN;
        c.mode         = GPIO_MODE_INPUT;
        c.pull_up_en   = GPIO_PULLUP_DISABLE;
        c.pull_down_en = GPIO_PULLDOWN_ENABLE;
        c.intr_type    = GPIO_INTR_DISABLE;     // polling + debounce, simple and reliable
        gpio_config(&c);
        if (!codec_ok_) { ESP_LOGW(TAG, "codec not ready; PTT task not started"); return; }
        xTaskCreate(&Es8311VoiceBoard::PttTaskEntry, "ptt_voice", 4096, this, 5, &ptt_task_);
        ESP_LOGI(TAG, "PTT ready: hold GPIO%d to talk, release to end", (int)BUTTON_BOOT_PIN);
    }

    static void PttTaskEntry(void* arg) { static_cast<Es8311VoiceBoard*>(arg)->PttLoop(); }

    // Hold to talk: press (GPIO0=0) starts capture, read 20ms frames in a loop and push_voice;
    // release (=1) calls voice_end.
    void PttLoop() {
        constexpr size_t kFrame = AUDIO_SAMPLE_RATE / 50;  // 20ms @ 16k = 320 samples
        int16_t buf[kFrame];
        bool talking = false;
        while (true) {
            const int pressed = (gpio_get_level(BUTTON_BOOT_PIN) == 1);  // high = pressed (active-high button)
            if (!talking) {
                if (!pressed) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
                vTaskDelay(pdMS_TO_TICKS(20));                     // debounce
                if (gpio_get_level(BUTTON_BOOT_PIN) != 1) continue; // bounce, ignore
                if (agent_link_state() != AGENT_STATE_READY) {
                    ESP_LOGW(TAG, "PTT pressed but Agent not connected, ignoring");
                    while (gpio_get_level(BUTTON_BOOT_PIN) == 1) vTaskDelay(pdMS_TO_TICKS(50));
                    continue;                                      // wait for release to avoid log spam
                }
                if (codec_.StartMic() != ESP_OK) { ESP_LOGE(TAG, "mic start failed"); continue; }
                talking = true;
                ESP_LOGI(TAG, "PTT pressed, starting voice uplink");
            } else {
                // wrap up if the link dropped
                if (agent_link_state() != AGENT_STATE_READY) {
                    agent_link_voice_end(); codec_.StopMic(); talking = false;
                    ESP_LOGW(TAG, "connection lost, ending voice");
                    continue;
                }
                size_t got = 0;
                if (codec_.ReadPcm(buf, kFrame, &got) == ESP_OK && got > 0) {  // blocks ~20ms
                    agent_link_push_voice(reinterpret_cast<const uint8_t*>(buf), got * sizeof(int16_t));
                }
                if (gpio_get_level(BUTTON_BOOT_PIN) != 1) {        // released
                    vTaskDelay(pdMS_TO_TICKS(20));                 // debounce
                    if (gpio_get_level(BUTTON_BOOT_PIN) != 1) {
                        agent_link_voice_end(); codec_.StopMic(); talking = false;
                        ESP_LOGI(TAG, "PTT released, ending voice");
                    }
                }
            }
        }
    }

    static constexpr size_t kPlayBufBytes = 16 * 1024;  // ~0.5s @ 16k to absorb bursts; drops when full

    Es8311Codec          codec_;
    bool                 codec_ok_  = false;
    StreamBufferHandle_t play_buf_  = nullptr;
    TaskHandle_t         play_task_ = nullptr;
    TaskHandle_t         ptt_task_  = nullptr;
};

DECLARE_BOARD(Es8311VoiceBoard);
