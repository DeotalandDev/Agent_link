// agent_link WiFi transport backend — skeleton/placeholder (contract first; see docs/PLAN.md P2-P4).
//
// The WiFi stack is an independent transport, peer to BLE: the device joins WiFi
// and talks to the cloud directly, carrying:
//   - Control commands: WebSocket / WebRTC DataChannel (frames reuse protocol.cpp)
//   - Voice up/down:     WebRTC audio track (Opus)
//   - Video up/down:     WebRTC video track (H.264/MJPEG)
// For now this file only provides empty agent_transport_t methods plus the
// parameter/callback setters, so firmware, App, and cloud can agree on one
// contract and build in parallel; start/send_* currently return "not supported".
#include "agent_link_transport.h"

#include "esp_log.h"

namespace {
constexpr const char* TAG = "agent_link.wifi";

// Connection parameters (loaded via agent_transport_wifi_set_config; used to join WiFi + reach the cloud once implemented).
const struct agent_wifi_config_s* s_cfg = nullptr;

// Transport -> core uplink callbacks (same shape as the BLE backend so the core wires both the same way).
void (*s_on_recv)(const uint8_t*, size_t) = nullptr;
void (*s_on_conn)(bool) = nullptr;
void (*s_on_stream)(agent_stream_t, const uint8_t*, size_t) = nullptr;

esp_err_t wifi_start(void* /*impl*/) {
    ESP_LOGW(TAG, "WiFi transport not implemented yet (PLAN.md P2). cfg=%s",
             s_cfg ? "set" : "null");
    (void)s_on_recv; (void)s_on_conn; (void)s_on_stream;
    return ESP_ERR_NOT_SUPPORTED;
}
void wifi_stop(void* /*impl*/) {}

// Control frames: will go over WebSocket / DataChannel once implemented.
esp_err_t wifi_send_ctrl(void* /*impl*/, const uint8_t* /*frame*/, size_t /*len*/) {
    return ESP_ERR_NOT_SUPPORTED;
}
// Data-plane session lifecycle: will start/stop the matching WebRTC track once implemented.
esp_err_t wifi_stream_start(void* /*impl*/, agent_stream_t /*type*/,
                            const uint8_t* /*meta*/, size_t /*meta_len*/) {
    return ESP_ERR_NOT_SUPPORTED;
}
// Media stream: will route by type to the matching WebRTC track (voice/video) once implemented.
esp_err_t wifi_send_stream(void* /*impl*/, agent_stream_t /*type*/,
                           const uint8_t* /*data*/, size_t /*len*/) {
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t wifi_stream_end(void* /*impl*/, agent_stream_t /*type*/, bool /*complete*/) {
    return ESP_ERR_NOT_SUPPORTED;
}
bool wifi_is_ready(void* /*impl*/) { return false; }

agent_transport_t s_wifi = {
    wifi_start, wifi_stop, wifi_send_ctrl,
    wifi_stream_start, wifi_send_stream, wifi_stream_end,
    wifi_is_ready, nullptr,
};
}  // namespace

extern "C" agent_transport_t* agent_transport_wifi(void) { return &s_wifi; }

extern "C" void agent_transport_wifi_set_config(const struct agent_wifi_config_s* cfg) {
    s_cfg = cfg;
}
extern "C" void agent_transport_wifi_set_recv(void (*cb)(const uint8_t*, size_t)) { s_on_recv = cb; }
extern "C" void agent_transport_wifi_set_conn(void (*cb)(bool)) { s_on_conn = cb; }
extern "C" void agent_transport_wifi_set_stream_recv(void (*cb)(agent_stream_t, const uint8_t*, size_t)) {
    s_on_stream = cb;
}
