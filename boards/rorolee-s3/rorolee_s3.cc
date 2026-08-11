// RoRoLee-S3 (ESP32-S3)

#include "board.h"
#include "config.h"
#include "sh8501_panel.h"
#include "es_codec.h"
#include "bq27220.h"

#include "agent_link.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#define TAG "RoRoLeeS3"

class RoRoLeeS3Board : public Board {
public:
    RoRoLeeS3Board() {
        PowerOnRail();
        InitDisplay();
        InitCodec();
        InitFuelGauge();
        InitButton();
    }

    const char* Name() const override { return "ROROLEE_S3"; }

    uint32_t Capabilities() const override {
        return AGENT_CAP_MIC | AGENT_CAP_SPEAKER | AGENT_CAP_SCREEN |
               AGENT_CAP_BUTTON | AGENT_CAP_HAPTIC | AGENT_CAP_BATTERY |
               AGENT_CAP_RECORDING;
    }

    // Agent asks the device to show text: there is no font rendering at this reference stage,
    // so a background-color change stands in for "text received".
    void ShowText(const char* utf8) override {
        ESP_LOGI(TAG, "ShowText \"%s\" (reference stage: blue fill for now; TODO font rendering)", utf8 ? utf8 : "");
        if (panel_.Ready()) panel_.FillSolid(rgb565::kBlue);
    }

    // Agent downlink TTS (PCM16/16k/mono): push into the play buffer and return immediately
    // without blocking (this callback runs in the BLE host task context and must never block).
    // ES8311 is written from the PlayLoop task. If the buffer is full (link burst > realtime), drop and warn.
    void PlayAudio(const uint8_t* pcm16, size_t bytes) override {
        if (!play_buf_ || !pcm16 || bytes == 0) return;
        const size_t sent = xStreamBufferSend(play_buf_, pcm16, bytes, 0);  // timeout 0 = non-blocking
        if (sent < bytes) {
            ESP_LOGW(TAG, "play buffer full — dropped %u/%u bytes", (unsigned)(bytes - sent), (unsigned)bytes);
        }
    }
    // End of a TTS segment: PlayLoop keeps playing whatever is left in the buffer (not flushed).
    void AudioEnd() override { ESP_LOGI(TAG, "AudioEnd (end of TTS segment; buffer plays out then goes silent)"); }

    void Vibrate(uint32_t ms) override {
        ESP_LOGI(TAG, "Vibrate %u ms (TODO: LEDC motor)", (unsigned)ms);
    }
    // Device -> Agent: battery / charging (app_main polls every 5s -> agent_link_report_battery -> event 0x14 + 0x2A19).
    int  GetBatteryLevel() override { return gauge_ok_ ? gauge_.Soc() : -1; }
    bool IsCharging()      override { return gauge_ok_ && gauge_.IsCharging(); }

private:
    void InitCodec() {
        EsCodecConfig cc = {};
        cc.i2c_port    = I2C_NUM_0;
        cc.pin_sda     = AUDIO_I2C_SDA;
        cc.pin_scl     = AUDIO_I2C_SCL;
        cc.pin_mclk    = AUDIO_I2S_MCLK;
        cc.pin_bclk    = AUDIO_I2S_BCLK;
        cc.pin_ws      = AUDIO_I2S_WS;
        cc.pin_din     = AUDIO_I2S_DIN;
        cc.pin_dout    = AUDIO_I2S_DOUT;
        cc.pin_pa_en   = AUDIO_PA_EN;
        cc.es7210_addr = ES7210_ADDR;
        cc.es8311_addr = ES8311_ADDR;
        cc.sample_rate = AUDIO_SAMPLE_RATE;
        cc.mic_gain    = 30;
        cc.out_volume  = 80;
        if (codec_.Init(cc) != ESP_OK) { ESP_LOGE(TAG, "codec init failed"); return; }
        codec_ok_ = true;

        play_buf_ = xStreamBufferCreate(kPlayBufBytes, /*trigger=*/1);
        if (!play_buf_) { ESP_LOGE(TAG, "play buffer alloc failed"); return; }
        xTaskCreate(&RoRoLeeS3Board::PlayTaskEntry, "spk_play", 4096, this, 5, &play_task_);
    }

    static void PlayTaskEntry(void* arg) { static_cast<RoRoLeeS3Board*>(arg)->PlayLoop(); }

    // Pull PCM from the play buffer and write it to the codec.
    void PlayLoop() {
        int16_t buf[256];
        while (true) {
            const size_t n = xStreamBufferReceive(play_buf_, buf, sizeof(buf), portMAX_DELAY);
            if (n >= sizeof(int16_t)) codec_.WritePcm(buf, n / sizeof(int16_t));
        }
    }

    // ── Fuel gauge (BQ27220) ──
    void InitFuelGauge() {
        if (!codec_ok_) { ESP_LOGW(TAG, "codec/I2C not ready; skipping fuel-gauge init"); return; }
        if (gauge_.Init(I2C_NUM_0, BQ27220_ADDR) != ESP_OK) {
            ESP_LOGW(TAG, "BQ27220 init failed, battery unavailable (GetBatteryLevel returns -1)");
            return;
        }
        gauge_ok_ = true;
        ESP_LOGI(TAG, "fuel gauge ready: SOC=%d%% charging=%d", gauge_.Soc(), (int)gauge_.IsCharging());
    }

    // GPIO0 (BOOT) button -> push-to-talk voice task
    void InitButton() {
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << BUTTON_BOOT_PIN;
        c.mode         = GPIO_MODE_INPUT;
        c.pull_up_en   = GPIO_PULLUP_ENABLE;    // BOOT key grounds when pressed -> high idle, low when pressed
        c.pull_down_en = GPIO_PULLDOWN_DISABLE;
        c.intr_type    = GPIO_INTR_DISABLE;     // polling + debounce, simple and reliable
        gpio_config(&c);
        if (!codec_ok_) { ESP_LOGW(TAG, "codec not ready; PTT task not started"); return; }
        xTaskCreate(&RoRoLeeS3Board::PttTaskEntry, "ptt_voice", 4096, this, 5, &ptt_task_);
        ESP_LOGI(TAG, "PTT ready: hold GPIO%d to talk, release to end", (int)BUTTON_BOOT_PIN);
    }

    static void PttTaskEntry(void* arg) { static_cast<RoRoLeeS3Board*>(arg)->PttLoop(); }

    // Hold to talk: press (GPIO0=0) starts capture, read 20ms frames in a loop and push_voice; release (=1) calls voice_end.
    void PttLoop() {
        constexpr size_t kFrame = AUDIO_SAMPLE_RATE / 50;  // 20ms @ 16k = 320 samples
        int16_t buf[kFrame];
        bool talking = false;
        while (true) {
            const int pressed = (gpio_get_level(BUTTON_BOOT_PIN) == 0);  // low = pressed
            if (!talking) {
                if (!pressed) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
                vTaskDelay(pdMS_TO_TICKS(20));                     // debounce
                if (gpio_get_level(BUTTON_BOOT_PIN) != 0) continue; // bounce, ignore
                if (agent_link_state() != AGENT_STATE_READY) {
                    ESP_LOGW(TAG, "PTT pressed but App not connected, ignoring");
                    while (gpio_get_level(BUTTON_BOOT_PIN) == 0) vTaskDelay(pdMS_TO_TICKS(50));
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
                if (gpio_get_level(BUTTON_BOOT_PIN) != 0) {        // released
                    vTaskDelay(pdMS_TO_TICKS(20));                 // debounce
                    if (gpio_get_level(BUTTON_BOOT_PIN) != 0) {
                        agent_link_voice_end(); codec_.StopMic(); talking = false;
                        ESP_LOGI(TAG, "PTT released, ending voice");
                    }
                }
            }
        }
    }


    // Power on the peripheral rail.
    void PowerOnRail() {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << POWER_CTRL_PIN;
        cfg.mode         = GPIO_MODE_OUTPUT;
        if (gpio_config(&cfg) != ESP_OK) { ESP_LOGE(TAG, "power gpio cfg failed"); return; }
        gpio_set_level(POWER_CTRL_PIN, POWER_CTRL_ACTIVE_HIGH ? 1 : 0);  // active low
        (void)gpio_hold_en(POWER_CTRL_PIN);
        vTaskDelay(pdMS_TO_TICKS(50));   // wait for the supply to settle
        ESP_LOGI(TAG, "peripheral power on (GPIO%d)", (int)POWER_CTRL_PIN);
    }
    // Initialize the display.
    void InitDisplay() {
        Sh8501Config c = {};
        c.spi_host = DISPLAY_SPI_HOST;
        c.pin_sck  = DISPLAY_SCK_PIN;
        c.pin_mosi = DISPLAY_MOSI_PIN;
        c.pin_cs   = DISPLAY_CS_PIN;
        c.pin_dc   = DISPLAY_DC_PIN;
        c.pin_rst  = DISPLAY_RST_PIN;
        c.width    = DISPLAY_WIDTH;
        c.height   = DISPLAY_HEIGHT;
        c.pclk_hz  = DISPLAY_SPI_CLK_HZ;
        c.spi_mode = DISPLAY_SPI_MODE;
        if (panel_.Init(c) != ESP_OK) {
            ESP_LOGE(TAG, "display init failed");
            return;
        }
        // Boot self-test: red/green/blue/white in turn, to eyeball that the panel lights up with no dead pixels or color cast.
        const uint16_t seq[] = { rgb565::kRed, rgb565::kGreen, rgb565::kBlue, rgb565::kWhite };
        for (uint16_t color : seq) {
            panel_.FillSolid(color);
            vTaskDelay(pdMS_TO_TICKS(400));
        }
        panel_.FillSolid(rgb565::kWhite);   // end on white as the idle screen (a black AMOLED looks like it's off); the Agent's content overwrites it later
        ESP_LOGI(TAG, "display lit + self-test done");
    }

    static constexpr size_t kPlayBufBytes = 16 * 1024;  // play buffer ~0.5s @ 16k to absorb bursts; drops when full

    Sh8501Panel         panel_;
    EsCodec             codec_;
    bool                codec_ok_  = false;
    Bq27220             gauge_;
    bool                gauge_ok_  = false;
    TaskHandle_t        ptt_task_  = nullptr;
    StreamBufferHandle_t play_buf_ = nullptr;
    TaskHandle_t        play_task_ = nullptr;
};

DECLARE_BOARD(RoRoLeeS3Board);
