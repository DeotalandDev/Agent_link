// ES8311 ASR (ESP32-S3) — minimal real-time ASR example board
//
// One button, two gestures:
//   Long-press >=1s : toggle mic upload (agent_link_asr_*). While on, 20ms PCM16 frames stream to the
//                       App over L2CAP and the App transcribes them live (ASR). Long-press again to stop.
//   Short-press     : while streaming, toggle "forward to agent" and tell the App via a board-private
//                       event (0x64, payload [subtype=0x01][state][seq LE]). state=1 -> App forwards the
//                       audio to the agent; state=0 -> App only transcribes locally

#include "board.h"
#include "config.h"
#include "es8311_audio.h"

#include "agent_link.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "Es8311Asr"

class Es8311AsrBoard : public Board {
public:
    Es8311AsrBoard() {
        InitCodec();
        InitButton();
    }

    const char* Name() const override { return "ROROLEE_ES8311_ASR"; }

    // A mic + button that streams audio up the record-stream channel the App transcribes (AGENT_CAP_RECORDING).
    uint32_t Capabilities() const override {
        return AGENT_CAP_MIC | AGENT_CAP_BUTTON | AGENT_CAP_RECORDING;
    }

private:
    // Audio codec (ES8311) — mic capture only; the DAC/output is left unused on this board
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
        c.out_volume  = 0;                // output unused -> keep the DAC muted
        if (codec_.Init(c) != ESP_OK) { ESP_LOGE(TAG, "codec init failed"); return; }
        codec_ok_ = true;
    }

    // Button (active-HIGH on this board: idles low, pressed drives high)
    void InitButton() {
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << BUTTON_BOOT_PIN;
        c.mode         = GPIO_MODE_INPUT;
        c.pull_up_en   = GPIO_PULLUP_DISABLE;
        c.pull_down_en = GPIO_PULLDOWN_ENABLE;
        c.intr_type    = GPIO_INTR_DISABLE;     // polling + debounce
        gpio_config(&c);
        if (!codec_ok_) { ESP_LOGW(TAG, "codec not ready; ASR task not started"); return; }
        xTaskCreate(&Es8311AsrBoard::AsrTaskEntry, "asr_btn", 4096, this, 5, &asr_task_);
        ESP_LOGI(TAG, "ASR ready on GPIO%d: long-press 1s = start/stop upload, short-press = toggle forward-to-agent",
                 (int)BUTTON_BOOT_PIN);
    }

    static void AsrTaskEntry(void* arg) { static_cast<Es8311AsrBoard*>(arg)->AsrLoop(); }

    // Button polarity lives ONLY here: pressed is HIGH on this board
    static bool BtnPressed() { return gpio_get_level(BUTTON_BOOT_PIN) == 1; }

    // Long-press action: toggle the ASR upload stream. On (re)start, forward-to-agent resets to OFF.
    void ToggleStreaming(bool& streaming, bool& forward) {
        if (!streaming) {
            if (agent_link_state() != AGENT_STATE_READY) {
                ESP_LOGW(TAG, "long-press but not connected, ignoring"); return;
            }
            if (agent_link_asr_start("asr") != ESP_OK) {   // needs the App's L2CAP CoC (PSM 0x0081) open
                ESP_LOGW(TAG, "asr_start failed — App must open the L2CAP channel (PSM 0x0081) first"); return;
            }
            if (codec_.StartMic() != ESP_OK) {
                ESP_LOGE(TAG, "mic start failed"); agent_link_asr_end(false); return;
            }
            streaming = true; forward = false;
            ESP_LOGI(TAG, "ASR streaming started (long-press); forward-to-agent OFF");
        } else {
            agent_link_asr_end(true); codec_.StopMic();
            streaming = false; forward = false;
            ESP_LOGI(TAG, "ASR streaming stopped (long-press)");
        }
    }

    // Short-press action: flip forward-to-agent and tell the App via a board-private event (0x64).
    // payload = [subtype=0x01][state 0/1][seq(4 LE)]; explicit state so a dropped frame can't desync.
    void SendForwardToggle(bool& forward, uint32_t& seq) {
        forward = !forward;
        ++seq;
        const uint8_t p[6] = {
            0x01,                                                   // subtype: agent-forward toggle
            static_cast<uint8_t>(forward ? 1 : 0),                  // state
            static_cast<uint8_t>(seq & 0xFF), static_cast<uint8_t>((seq >> 8) & 0xFF),
            static_cast<uint8_t>((seq >> 16) & 0xFF), static_cast<uint8_t>((seq >> 24) & 0xFF),
        };
        const esp_err_t r = agent_link_push_event(AGENT_EVT_CUSTOM, p, sizeof(p));
        ESP_LOGI(TAG, "forward-to-agent %s (event 0x64 %s, seq=%u)",
                 forward ? "ON" : "OFF", r == ESP_OK ? "sent" : "send-failed", (unsigned)seq);
    }

    // Long-press toggles the upload stream; short-press toggles forward-to-agent. The mic read (~20ms/frame)
    // doubles as the button poll tick while streaming; when idle we poll on a plain delay.
    void AsrLoop() {
        constexpr size_t kFrame = AUDIO_SAMPLE_RATE / 50;   // 20ms @ 16k = 320 samples
        const TickType_t kLong  = pdMS_TO_TICKS(1000);      // long-press threshold
        const TickType_t kPoll  = pdMS_TO_TICKS(20);        // idle button poll tick
        int16_t buf[kFrame];

        bool       streaming  = false;   // ASR upload active (long-press toggles)
        bool       forward    = false;   // App should forward audio to the agent (short-press toggles)
        uint32_t   seq        = 0;       // forward-toggle event sequence
        bool       was_pressed = false;
        TickType_t press_tick   = 0;
        bool       long_fired   = false; // long-press already acted on during this hold

        while (true) {
            // ── 1) Button edges: long-press fires on the hold crossing 1s; short-press fires on release ──
            const bool pressed = BtnPressed();
            if (pressed && !was_pressed) {                          // press edge
                press_tick = xTaskGetTickCount();
                long_fired = false;
            } else if (pressed && was_pressed) {                    // held
                if (!long_fired && (xTaskGetTickCount() - press_tick) >= kLong) {
                    long_fired = true;
                    ToggleStreaming(streaming, forward);
                }
            } else if (!pressed && was_pressed) {                   // release edge
                if (!long_fired) {                                  // a short click (long-press already handled)
                    if (streaming) SendForwardToggle(forward, seq);
                    else ESP_LOGW(TAG, "short-press ignored (long-press 1s to start streaming first)");
                }
            }
            was_pressed = pressed;

            // ── 2) While streaming: pump one 20ms mic frame up the ASR channel ──
            if (streaming) {
                if (agent_link_state() != AGENT_STATE_READY) {      // link dropped mid-stream
                    agent_link_asr_end(false); codec_.StopMic();
                    streaming = false; forward = false;
                    ESP_LOGW(TAG, "connection lost, ending ASR");
                    continue;
                }
                size_t got = 0;
                if (codec_.ReadPcm(buf, kFrame, &got) == ESP_OK && got > 0) {  // blocks ~20ms -> also the poll tick
                    const esp_err_t r = agent_link_asr_push(reinterpret_cast<const uint8_t*>(buf), got * sizeof(int16_t));
                    if (r == ESP_ERR_NO_MEM) {                      // truncated by backpressure -> stop this segment
                        ESP_LOGW(TAG, "ASR truncated (link can't keep up), stopping");
                        agent_link_asr_end(false); codec_.StopMic();
                        streaming = false; forward = false;
                    }
                } else {
                    vTaskDelay(kPoll);                              // read failed: keep the poll cadence
                }
            } else {
                vTaskDelay(kPoll);                                  // idle: just poll the button
            }
        }
    }

    Es8311Codec  codec_;
    bool         codec_ok_ = false;
    TaskHandle_t asr_task_ = nullptr;
};

DECLARE_BOARD(Es8311AsrBoard);
