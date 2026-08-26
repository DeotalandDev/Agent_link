#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "agent_link.h"
#include "board.h"

namespace {
constexpr const char* TAG = "agent_link.app";

void on_audio_out(const uint8_t* pcm16, size_t bytes, void*) { Board::GetInstance().PlayAudio(pcm16, bytes); }
void on_audio_end(void*) { Board::GetInstance().AudioEnd(); }
void on_show_text(const char* utf8, void*) { Board::GetInstance().ShowText(utf8); }
void on_haptic(uint32_t duration_ms, void*) { Board::GetInstance().Vibrate(duration_ms); }
void on_led(uint32_t rgb, void*) { Board::GetInstance().SetLed(rgb); }

void on_state(agent_state_t state, void*) {
    ESP_LOGI(TAG, "[state] %s",
             state == AGENT_STATE_READY     ? "READY" :
             state == AGENT_STATE_CONNECTED ? "CONNECTED" : "DISCONNECTED");
}
}  // namespace

extern "C" void app_main(void) {
    Board& board = Board::GetInstance();
    ESP_LOGI(TAG, "board = %s, caps = 0x%04x", board.Name(), static_cast<unsigned>(board.Capabilities()));

    //Callback function registration
    agent_output_cb_t out = {};
    out.on_audio_out = on_audio_out;
    out.on_audio_end = on_audio_end;
    out.on_show_text = on_show_text;
    out.on_haptic    = on_haptic;
    out.on_led       = on_led;

    agent_link_config_t cfg = {};
    cfg.device_name = board.Name();
    cfg.caps        = board.Capabilities();
    cfg.output      = &out;
    cfg.on_state    = on_state;
#if CONFIG_AGENT_LINK_TRANSPORT_WIFI
    cfg.transport   = AGENT_TRANSPORT_WIFI;  // WiFi station + captive-portal provisioning
#endif

    ESP_ERROR_CHECK(agent_link_init(&cfg));     //agent_link initialization
    ESP_ERROR_CHECK(agent_link_start());

    while (true) {
        if (agent_link_state() == AGENT_STATE_READY) {
            const int batt = board.GetBatteryLevel();
            if (batt >= 0) {
                agent_link_report_battery(static_cast<uint8_t>(batt), board.IsCharging());
            }
            ESP_LOGD(TAG, "battery poll (batt=%d)", batt);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
