// agent_link core — transport-agnostic capability routing + lifecycle.
//
// One API, two transports: init selects BLE or WiFi backend (s_tx) based on cfg.transport,
// and both wire the upstream callbacks to the same OnCtrlFrame/OnConn, so upper-layer
// push_*/on_* are transport-independent.
//
// Control plane (already wired):
//   - start()   -> s_tx->start() (BLE: NimBLE advertising + GATT Service C 0xFFC0).
//   - downlink commands -> transport receives write -> OnCtrlFrame decodes ->
//     on_custom (device-specific commands) + automatic ACK.
//   - uplink events -> report_battery etc. -> protocol frames -> s_tx->send_ctrl
//     (notify 0xFFC4 events / 0xFFC1 responses).
//   - connection state -> OnConn -> on_state.
// Data plane (partially wired):
//   - voice uplink push_voice/voice_end -> s_tx->stream_start/send_stream/stream_end.
//     BLE backend: GATT Notify 0xFFA1, event 0x40 VoiceChunk (see transport_ble.cpp voice uploader).
//   - generic I/O (register_io/push_reading/actuate) — self-describing manifest (event 0x18),
//     reading reports (event 0x19), actuator downlink (command 0x33), manifest fetch (command 0x34).
//     See docs/device-io.md.
// To be wired: recording_* / video push — needs L2CAP (BLE) / WebRTC (WiFi).
#include "agent_link.h"
#include "agent_link_transport.h"
#include "protocol.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "esp_log.h"
#include "esp_timer.h"

namespace {
constexpr const char* TAG = "agent_link";

agent_link_config_t s_cfg   = {};
agent_output_cb_t   s_out   = {};
bool                s_have_out = false;
agent_state_t       s_state = AGENT_STATE_DISCONNECTED;
agent_transport_t*  s_tx    = nullptr;
bool                s_voice_started = false;  // whether a voice session is currently open
bool                s_asr_started   = false;  // whether an ASR / record-stream session is currently open

// The SDK only notifies when the battery level changes / charging state changes / a
// low-battery edge occurs. The cache is updated only on a successful send; a failed
// send is retried on the next call.
int  s_bat_pct = -1;          // last successfully pushed battery level (-1 = unknown: force a resend after connect for initial sync)
int  s_bat_chg = -1;          // last successfully pushed charging state (-1 = unknown; otherwise 0/1)
bool s_bat_low_armed = true;  // low-battery edge armed (true = not yet triggered)
constexpr uint8_t kBatLowThreshold = 20;  // low-battery threshold (%)

// Generic I/O endpoint registry (manifest + reading reports + downlink actuators).
// Downlink actuator commands are routed by id to the matching cb.
constexpr int kMaxIo = 32;
struct IoEntry {
    const agent_link_io_desc_t* desc;
    agent_io_actuate_cb_t       cb;
    void*                       ctx;
    // Reading cache + reporting policy (see 0x35 GetReading / 0x36 SetReadingConfig).
    uint8_t  policy_mode;      // 0=passthrough(default) 1=off 2=periodic(rate) 3=on-change
    uint16_t policy_rate_hz;   // target rate when policy_mode == periodic
    int64_t  last_sent_us;     // last time a reading was actually sent (for throttling)
    int64_t  last_ts_us;       // last time push_reading was called (for 0x35 age_ms)
    uint8_t  last_val[16];     // cached last value (fixed-size types <= 16B; for 0x35 + dedup)
    uint8_t  last_len;         // cached value length (0 = none cached)
    bool     has_last;         // whether last_val / last_ts_us are valid
};

IoEntry s_io[kMaxIo] = {};
int     s_io_count = 0;
bool    s_manifest_sent = false;  // whether the manifest has been sent in this connection
uint32_t s_manifest_rev = 1;      // manifest revision; bumped by agent_link_notify_manifest_changed()

// Per-endpoint reporting policy values (0x36 SetReadingConfig). Default 0 = passthrough so a
// zero-initialized IoEntry keeps the pre-existing "forward every push_reading" behavior.
enum { IO_POLICY_PASSTHROUGH = 0, IO_POLICY_OFF = 1, IO_POLICY_PERIODIC = 2, IO_POLICY_ONCHANGE = 3 };

// Single exit point for data-plane calls that are not wired yet (push_voice / recording_*).
esp_err_t not_ready(const char* what) {
    if (s_tx && s_tx->is_ready && s_tx->is_ready(s_tx->impl)) {
        ESP_LOGD(TAG, "%s: transport ready but not wired yet (data plane TODO)", what);
        return ESP_ERR_NOT_SUPPORTED;
    }
    ESP_LOGD(TAG, "%s: link not ready — ignored", what);
    return ESP_ERR_INVALID_STATE;
}

// ── Generic I/O: manifest serialization + reading/actuator routing ──────────────
// Look up an endpoint by id.
const IoEntry* FindIo(const char* id) {
    if (!id) return nullptr;
    for (int i = 0; i < s_io_count; ++i)
        if (s_io[i].desc && s_io[i].desc->id && strcmp(s_io[i].desc->id, id) == 0)
            return &s_io[i];
    return nullptr;
}

// Look up an endpoint index by id (-1 if not found); used where the entry must be mutated.
int FindIoIndex(const char* id) {
    if (!id) return -1;
    for (int i = 0; i < s_io_count; ++i)
        if (s_io[i].desc && s_io[i].desc->id && strcmp(s_io[i].desc->id, id) == 0)
            return i;
    return -1;
}

// Fixed byte size of a value type; BLOB is variable-length and returns -1. Used for push_reading length checks.
int IoValueSize(agent_val_t t) {
    switch (t) {
    case AGENT_VAL_BOOL: return 1;
    case AGENT_VAL_U16:  return 2;
    case AGENT_VAL_I32:  return 4;
    case AGENT_VAL_F32:  return 4;
    case AGENT_VAL_RGB:  return 4;
    case AGENT_VAL_VEC2: return 8;
    case AGENT_VAL_VEC3: return 12;
    case AGENT_VAL_BLOB: return -1;
    case AGENT_VAL_STR:  return -1;
    }
    return -1;
}

// Value type -> the type string used in the manifest.
const char* ValTypeName(agent_val_t t) {
    switch (t) {
    case AGENT_VAL_BOOL: return "bool";
    case AGENT_VAL_U16:  return "u16";
    case AGENT_VAL_I32:  return "i32";
    case AGENT_VAL_F32:  return "f32";
    case AGENT_VAL_RGB:  return "rgb";
    case AGENT_VAL_VEC2: return "vec2";
    case AGENT_VAL_VEC3: return "vec3";
    case AGENT_VAL_BLOB: return "blob";
    case AGENT_VAL_STR:  return "str";
    }
    return "blob";
}

// Escape a string for JSON (UTF-8 multibyte bytes >= 0x80 pass through unchanged, which is valid JSON).
void JsonEsc(std::string& out, const char* s) {
    if (!s) return;
    for (const char* p = s; *p; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:
            if (c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); out += b; }
            else out += static_cast<char>(c);
        }
    }
}

// ── D1: rich outputs (LED / haptic / screen text) exposed as generic OUT endpoints ──
// The rich callbacks (on_led / on_haptic / on_show_text) had no downlink command that reached
// them. The SDK now auto-registers a synthetic OUT endpoint for each rich capability the board
// actually wired, so the Agent drives them through the same 0x33 IoActuate path as any actuator.
// Board code is unchanged — it just keeps providing the callbacks.
void SynthLedCb(const char* /*id*/, const uint8_t* args, size_t len, void* /*ctx*/) {
    if (len < 4 || !(s_have_out && s_out.on_led)) return;
    const uint32_t rgb = static_cast<uint32_t>(args[0]) | (static_cast<uint32_t>(args[1]) << 8) |
                         (static_cast<uint32_t>(args[2]) << 16) | (static_cast<uint32_t>(args[3]) << 24);
    s_out.on_led(rgb, s_out.ctx);
}
void SynthHapticCb(const char* /*id*/, const uint8_t* args, size_t len, void* /*ctx*/) {
    if (len < 2 || !(s_have_out && s_out.on_haptic)) return;
    const uint32_t ms = static_cast<uint32_t>(args[0]) | (static_cast<uint32_t>(args[1]) << 8);
    s_out.on_haptic(ms, s_out.ctx);
}
void SynthScreenCb(const char* /*id*/, const uint8_t* args, size_t len, void* /*ctx*/) {
    if (!(s_have_out && s_out.on_show_text)) return;
    char buf[256];
    const size_t n = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, args, n);
    buf[n] = '\0';
    s_out.on_show_text(buf, s_out.ctx);
}

// Static descriptors (register_io stores the pointer, so these must outlive the call).
const agent_link_io_desc_t kSynthLed = {
    .id = "led0", .dir = AGENT_IO_OUT, .kind = "led", .value = AGENT_VAL_RGB,
    .desc = "status LED color, 0x00RRGGBB",
};
const agent_link_io_desc_t kSynthHaptic = {
    .id = "motor0", .dir = AGENT_IO_OUT, .kind = "haptic", .value = AGENT_VAL_U16,
    .unit = "ms", .desc = "vibration motor, duration in ms",
};
const agent_link_io_desc_t kSynthScreen = {
    .id = "screen0", .dir = AGENT_IO_OUT, .kind = "screen.text", .value = AGENT_VAL_STR,
    .desc = "text display",
};

// Register synthetic endpoints for whichever rich outputs the board wired + advertised via caps.
void RegisterSyntheticEndpoints() {
    if (!s_have_out) return;
    if ((s_cfg.caps & AGENT_CAP_LED)    && s_out.on_led       && !FindIo("led0"))
        agent_link_register_io(&kSynthLed, SynthLedCb, nullptr);
    if ((s_cfg.caps & AGENT_CAP_HAPTIC) && s_out.on_haptic    && !FindIo("motor0"))
        agent_link_register_io(&kSynthHaptic, SynthHapticCb, nullptr);
    if ((s_cfg.caps & AGENT_CAP_SCREEN) && s_out.on_show_text && !FindIo("screen0"))
        agent_link_register_io(&kSynthScreen, SynthScreenCb, nullptr);
}

// Serialize the manifest as an object envelope {proto, rev, caps, io:[...]} (see docs/agent_link_ble.md §6.2).
std::string BuildManifestJson() {
    std::string s = "{\"proto\":";
    s += std::to_string(AGENT_LINK_PROTO_VERSION);
    s += ",\"rev\":";  s += std::to_string(s_manifest_rev);
    s += ",\"caps\":"; s += std::to_string(static_cast<unsigned>(s_cfg.caps));
    s += ",\"io\":[";
    bool first = true;
    for (int i = 0; i < s_io_count; ++i) {
        const agent_link_io_desc_t* d = s_io[i].desc;
        if (!d || !d->id || !d->kind) continue;
        if (!first) s += ",";
        first = false;
        s += "{\"id\":\"";     JsonEsc(s, d->id);
        s += "\",\"dir\":\"";  s += (d->dir == AGENT_IO_OUT ? "out" : "in");
        s += "\",\"kind\":\""; JsonEsc(s, d->kind);
        s += "\",\"value\":\""; s += ValTypeName(d->value); s += "\"";
        if (d->unit && d->unit[0])                 { s += ",\"unit\":\""; JsonEsc(s, d->unit); s += "\""; }
        if (d->desc && d->desc[0])                 { s += ",\"desc\":\""; JsonEsc(s, d->desc); s += "\""; }
        if (d->display_name && d->display_name[0]) { s += ",\"name\":\""; JsonEsc(s, d->display_name); s += "\""; }
        if (d->range_min != d->range_max) {
            char b[64];
            snprintf(b, sizeof b, ",\"range\":[%g,%g]",
                     static_cast<double>(d->range_min), static_cast<double>(d->range_max));
            s += b;
        }
        if (d->rate_hz) {
            char b[32];
            snprintf(b, sizeof b, ",\"rate_hz\":%u", static_cast<unsigned>(d->rate_hz));
            s += b;
        }
        if (d->event == AGENT_EVT_ON_CHANGE)       s += ",\"event\":\"change\"";
        else if (d->event == AGENT_EVT_THRESHOLD)  s += ",\"event\":\"threshold\"";
        if (d->audience == AGENT_AUD_USER)         s += ",\"audience\":\"user\"";
        if (d->enum_json && d->enum_json[0])       { s += ",\"enum\":"; s += d->enum_json; }
        if (d->default_json && d->default_json[0]) { s += ",\"default\":"; s += d->default_json; }
        if (d->dir == AGENT_IO_OUT && d->args_schema && d->args_schema[0]) {
            s += ",\"args\":"; s += d->args_schema;   // args_schema is itself JSON
        }
        s += "}";
    }
    s += "]}";
    return s;
}

// Send the manifest, fragmented into 0x18 IoManifest events over the control plane (send_ctrl).
//   Each chunk payload = [chunk_idx(1)] [last(1: 0/1)] [json fragment...]; the App concatenates
//   chunks in order until last=1, then parses the whole JSON.
// Triggered: after the BLE peer subscribes to 0xFFC4 (OnLinkReady) / after WiFi connects (OnConn) /
//   on the App's 0x34 fetch command.
void SendManifest(bool force) {
    if (s_io_count == 0) return;                    // no endpoints: no manifest
    if (!force && s_manifest_sent) return;          // already sent in this connection
    if (!s_tx || !s_tx->send_ctrl) return;

    const std::string json = BuildManifestJson();

    // Chunk budget: a BLE notify value is <= ATT_MTU-3, minus 6 (frame header) + 2 (chunk header). WiFi has no such limit.
    size_t budget;
    if (s_cfg.transport == AGENT_TRANSPORT_WIFI) {
        budget = 1024;
    } else {
        const uint16_t mtu = agent_transport_ble_att_mtu();
        budget = (mtu > 3 + 8 + 16) ? static_cast<size_t>(mtu - 3 - 8) : 150;
    }
    if (budget < 16)  budget = 16;
    if (budget > 480) budget = 480;

    const uint8_t* jb    = reinterpret_cast<const uint8_t*>(json.data());
    const size_t   total = json.size();
    size_t  off = 0;
    uint8_t idx = 0;
    bool    ok  = true;
    do {
        size_t n = total - off;
        if (n > budget) n = budget;
        std::vector<uint8_t> p;
        p.reserve(2 + n);
        p.push_back(idx);
        p.push_back((off + n >= total) ? 0x01 : 0x00);   // last flag
        p.insert(p.end(), jb + off, jb + off + n);
        auto ev = agentlink::BuildEvent(0x18, p.data(), p.size());
        if (s_tx->send_ctrl(s_tx->impl, ev.data(), ev.size()) != ESP_OK) { ok = false; break; }
        off += n;
        ++idx;
    } while (off < total);

    if (ok) {
        s_manifest_sent = true;
        ESP_LOGI(TAG, "manifest sent: %d endpoint(s), %uB in %u chunk(s)",
                 s_io_count, static_cast<unsigned>(total), static_cast<unsigned>(idx));
    } else {
        ESP_LOGW(TAG, "manifest send failed at chunk %u — App can re-fetch via command 0x34",
                 static_cast<unsigned>(idx));
    }
}

// BLE: fired after the peer subscribes to the event channel 0xFFC4 (notifications are only
// delivered from that point on) -> send the manifest.
void OnLinkReady() { SendManifest(/*force=*/false); }

// Transport -> core: connection state change.
void OnConn(bool connected) {
    // The protocol is plaintext with no encryption gate, so the link is usable as soon as it connects -> go READY.
    s_state = connected ? AGENT_STATE_READY : AGENT_STATE_DISCONNECTED;
    s_manifest_sent = false;      // reset on connect and disconnect: re-send after 0xFFC4 subscribe (BLE) / after connect (WiFi)
    if (!connected) {
        s_voice_started = false;  // on disconnect: the next push_voice reopens the session
        s_asr_started   = false;  // and the next asr_start reopens the ASR stream
    } else {
        // Clear the battery cache on connect so the next report_battery force-resends the current value (initial sync for the App).
        s_bat_pct = -1; s_bat_chg = -1; s_bat_low_armed = true;
        // WiFi: notifications work as soon as the WS connects -> send the manifest immediately.
        // BLE: must wait for the peer to subscribe to 0xFFC4; OnLinkReady sends it (otherwise the notify is dropped).
        if (s_cfg.transport == AGENT_TRANSPORT_WIFI) SendManifest(/*force=*/false);
    }
    if (s_cfg.on_state) s_cfg.on_state(s_state, s_cfg.state_ctx);
}

// Transport -> core: a control frame arrived (peer wrote the command channel 0xFFC1).
void OnCtrlFrame(const uint8_t* data, size_t len) {
    agentlink::Frame f;
    if (!agentlink::ParseFrame(data, len, f)) {
        ESP_LOGW(TAG, "bad ctrl frame (%u bytes)", static_cast<unsigned>(len));
        return;
    }
    if (f.msg_type != agentlink::kMsgCommand) return;  // the device side only handles "commands"

    ESP_LOGD(TAG, "cmd 0x%02X seq=%u payload=%uB",
             f.command_id, f.sequence, static_cast<unsigned>(f.payload.size()));

    // ── Command dispatch + response (may carry data) ────────────────────────────
    // Priority: SDK built-ins (standard commands with data) > device on_command (may return data) > on_custom (fire-and-forget).
    uint8_t  extra[128];
    size_t   extra_len = 0;
    uint8_t  status = 0;   // 0 = success
    uint16_t error  = 0;

    if (f.command_id == 0x03) {
        // 0x03 GetChargingStatus: answered by the SDK from cached battery. extra = [charging_state(0/1), level(0-100)].
        // The battery cache comes from report_battery (the board polls every 5s); payload must be empty, otherwise 1004.
        if (f.payload.empty()) {
            extra[0] = (s_bat_chg == 1) ? 0x01 : 0x00;
            extra[1] = (s_bat_pct >= 0 && s_bat_pct <= 100) ? static_cast<uint8_t>(s_bat_pct) : 0;
            extra_len = 2;
        } else {
            status = 1; error = 1004;  // InvalidPayload
        }
    } else if (f.command_id == 0x33) {
        // 0x33 IoActuate: payload = [id_len(1)][id(UTF-8)][args...], routed by id to the registered actuator cb.
        const std::vector<uint8_t>& pl = f.payload;
        char io_id[64];
        if (pl.empty() || pl[0] == 0 || pl[0] >= sizeof(io_id) || 1u + pl[0] > pl.size()) {
            status = 1; error = 1004;  // InvalidPayload
        } else {
            const uint8_t id_len = pl[0];
            memcpy(io_id, pl.data() + 1, id_len);
            io_id[id_len] = '\0';
            const IoEntry* e = FindIo(io_id);
            if (e && e->desc->dir == AGENT_IO_OUT && e->cb) {
                const uint8_t* args     = pl.data() + 1 + id_len;
                const size_t   args_len = pl.size() - 1 - id_len;
                e->cb(io_id, args, args_len, e->ctx);   // implementation must return quickly; do not block the transport task
                ESP_LOGI(TAG, "actuate '%s' (%uB args)", io_id, static_cast<unsigned>(args_len));
            } else {
                status = 1; error = 1003;  // BusinessFailed: unknown or non-actuatable id
                ESP_LOGW(TAG, "actuate '%s': no matching OUT endpoint/callback", io_id);
            }
        }
    } else if (f.command_id == 0x34) {
        // 0x34 GetIoManifest: App-initiated fetch -> (re)send the manifest (0x18), then reply with the usual ACK.
        SendManifest(/*force=*/true);
    } else if (f.command_id == 0x35) {
        // 0x35 GetReading: payload = [id_len][id]; reply extra = [val_type][age_ms(2,LE)][value] from cache.
        const std::vector<uint8_t>& pl = f.payload;
        char io_id[64];
        if (pl.empty() || pl[0] == 0 || pl[0] >= sizeof(io_id) || 1u + pl[0] > pl.size()) {
            status = 1; error = 1004;  // InvalidPayload
        } else {
            memcpy(io_id, pl.data() + 1, pl[0]);
            io_id[pl[0]] = '\0';
            const int idx = FindIoIndex(io_id);
            if (idx < 0 || s_io[idx].desc->dir != AGENT_IO_IN) {
                status = 1; error = 1003;              // unknown / non-readable id
            } else if (!s_io[idx].has_last) {
                status = 1; error = 1005;              // NoData: nothing cached yet
            } else {
                const int64_t age = (esp_timer_get_time() - s_io[idx].last_ts_us) / 1000;
                const uint16_t age_ms = (age < 0) ? 0 : (age > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(age));
                size_t k = 0;
                extra[k++] = static_cast<uint8_t>(s_io[idx].desc->value);
                extra[k++] = age_ms & 0xFF;
                extra[k++] = (age_ms >> 8) & 0xFF;
                memcpy(extra + k, s_io[idx].last_val, s_io[idx].last_len);
                k += s_io[idx].last_len;
                extra_len = k;
            }
        }
    } else if (f.command_id == 0x36) {
        // 0x36 SetReadingConfig: payload = [id_len][id][mode(1)][rate_hz(2,LE)]. Handled entirely in the SDK.
        const std::vector<uint8_t>& pl = f.payload;
        char io_id[64];
        if (pl.empty() || pl[0] == 0 || pl[0] >= sizeof(io_id) ||
            static_cast<size_t>(2 + pl[0]) > pl.size()) {
            status = 1; error = 1004;
        } else {
            const uint8_t id_len = pl[0];
            memcpy(io_id, pl.data() + 1, id_len);
            io_id[id_len] = '\0';
            const uint8_t mode = pl[1 + id_len];
            uint16_t rate = 0;
            if (static_cast<size_t>(4 + id_len) <= pl.size())
                rate = static_cast<uint16_t>(pl[2 + id_len]) | (static_cast<uint16_t>(pl[3 + id_len]) << 8);
            const int idx = FindIoIndex(io_id);
            if (idx < 0 || s_io[idx].desc->dir != AGENT_IO_IN) { status = 1; error = 1003; }
            else if (mode > IO_POLICY_ONCHANGE)                { status = 1; error = 1004; }
            else {
                s_io[idx].policy_mode    = mode;
                s_io[idx].policy_rate_hz = rate;
                s_io[idx].last_sent_us   = 0;
                ESP_LOGI(TAG, "reading config '%s': mode=%u rate=%uHz", io_id, mode, static_cast<unsigned>(rate));
            }
        }
    } else if (f.command_id == 0x3C) {
        // 0x3C StartCapture: payload = [mode(1)][max_ms(2,LE)] -> on_listen(true, max_ms). Requires MIC + on_listen.
        if (!(s_cfg.caps & AGENT_CAP_MIC) || !(s_have_out && s_out.on_listen)) {
            status = 1; error = 1002;                  // NotCapable
        } else {
            uint32_t max_ms = 0;
            if (f.payload.size() >= 3)
                max_ms = static_cast<uint32_t>(f.payload[1]) | (static_cast<uint32_t>(f.payload[2]) << 8);
            s_out.on_listen(true, max_ms, s_out.ctx);
        }
    } else if (f.command_id == 0x3D) {
        // 0x3D StopCapture -> on_listen(false, 0).
        if (s_have_out && s_out.on_listen) s_out.on_listen(false, 0, s_out.ctx);
    } else if (s_have_out && s_out.on_command &&
               s_out.on_command(f.command_id, f.payload.data(), f.payload.size(),
                                extra, sizeof(extra), &extra_len, s_out.ctx)) {
        // handled by the device (extra may carry response data); status stays 0.
    } else if (s_have_out && s_out.on_custom) {
        // escape hatch: fire-and-forget, reply with an empty ACK.
        s_out.on_custom(f.command_id, f.payload.data(), f.payload.size(), s_out.ctx);
    }

    // 0x05 VoiceReply: payload = session_id(4) + status(1). status=3 -> playback finished -> on_audio_end.
    // (status=2 carries an audio reply: the TTS then arrives over L2CAP, going through OnStreamData -> on_audio_out.)
    if (f.command_id == 0x05 && f.payload.size() >= 5) {
        if (f.payload[4] == 3 && s_have_out && s_out.on_audio_end) s_out.on_audio_end(s_out.ctx);
    }

    // Reply (request-response pairing); a query command carrying extra_data returns its data to the App here.
    auto resp = agentlink::BuildResponse(f.command_id, f.sequence, status, error,
                                         extra_len ? extra : nullptr, extra_len);
    if (s_tx && s_tx->send_ctrl) s_tx->send_ctrl(s_tx->impl, resp.data(), resp.size());
}

// Transport -> core: a data-plane chunk arrived (e.g. downlink TTS voice over L2CAP).
void OnStreamData(agent_stream_t type, const uint8_t* data, size_t len) {
    if (type == AGENT_STREAM_VOICE) {
        // Downlink TTS PCM -> hand to the board for playback. on_audio_out should enqueue quickly and not block (this runs in the transport task).
        if (s_have_out && s_out.on_audio_out) s_out.on_audio_out(data, len, s_out.ctx);
    }
    // Future downlink video/file: type == AGENT_STREAM_VIDEO -> on_video_out, etc.
}
}  // namespace

// ── Lifecycle ───────────────────────────────────────────────────────────────────
esp_err_t agent_link_init(const agent_link_config_t* cfg) {
    if (!cfg || !cfg->device_name) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (cfg->output) { s_out = *cfg->output; s_have_out = true; }

    // Transport backend selection ("two transports"). Both wire their uplink callbacks to the
    // same core (OnCtrlFrame/OnConn), so the upper-layer push_*/on_* stay transport-independent.
    switch (s_cfg.transport) {
    case AGENT_TRANSPORT_WIFI:
        s_tx = agent_transport_wifi();
        agent_transport_wifi_set_config(s_cfg.wifi);
        agent_transport_wifi_set_name(s_cfg.device_name);  // SoftAP SSID prefix for captive-portal provisioning
        agent_transport_wifi_set_recv(&OnCtrlFrame);
        agent_transport_wifi_set_conn(&OnConn);
        agent_transport_wifi_set_stream_recv(&OnStreamData);
        break;
    case AGENT_TRANSPORT_BOTH:  // TODO(P5): BLE control + WiFi media together; start with BLE for now.
    case AGENT_TRANSPORT_BLE:
    default:
        s_tx = agent_transport_ble();
        agent_transport_ble_set_name(s_cfg.device_name);  // advertising name = device name
        agent_transport_ble_set_recv(&OnCtrlFrame);       // control frame received -> parse and route
        agent_transport_ble_set_conn(&OnConn);            // connection state -> on_state
        agent_transport_ble_set_stream_recv(&OnStreamData); // L2CAP downlink TTS -> on_audio_out
        agent_transport_ble_set_ready(&OnLinkReady);        // after 0xFFC4 subscribe -> send the I/O manifest
        // Standard Device Information Service (0x180A): model defaults to device_name; NULL manufacturer/firmware keep the SDK defaults.
        agent_transport_ble_set_device_info(s_cfg.manufacturer,
                                            s_cfg.model ? s_cfg.model : s_cfg.device_name,
                                            s_cfg.firmware_rev);
        break;
    }

    // D1: expose wired rich outputs (LED / haptic / screen) as generic OUT endpoints in the manifest.
    RegisterSyntheticEndpoints();

    const char* tx_name = s_cfg.transport == AGENT_TRANSPORT_WIFI ? "wifi"
                        : s_cfg.transport == AGENT_TRANSPORT_BOTH ? "both(ble)" : "ble";
    ESP_LOGI(TAG, "init: name='%s' caps=0x%04x proto=v%d transport=%s io=%d",
             s_cfg.device_name, static_cast<unsigned>(s_cfg.caps), AGENT_LINK_PROTO_VERSION,
             tx_name, s_io_count);
    return ESP_OK;
}

esp_err_t agent_link_start(void) {
    ESP_LOGI(TAG,
             "start: caps=0x%04x out{audio=%d text=%d image=%d video=%d haptic=%d led=%d actuate=%d agentlist=%d command=%d custom=%d} io=%d",
             static_cast<unsigned>(s_cfg.caps),
             s_have_out && s_out.on_audio_out  ? 1 : 0,
             s_have_out && s_out.on_show_text  ? 1 : 0,
             s_have_out && s_out.on_show_image ? 1 : 0,
             s_have_out && s_out.on_video_out  ? 1 : 0,
             s_have_out && s_out.on_haptic     ? 1 : 0,
             s_have_out && s_out.on_led        ? 1 : 0,
             s_have_out && s_out.on_actuate    ? 1 : 0,
             s_have_out && s_out.on_agent_list ? 1 : 0,
             s_have_out && s_out.on_command    ? 1 : 0,
             s_have_out && s_out.on_custom     ? 1 : 0,
             s_io_count);
    // Start the transport backend (BLE: NimBLE advertising + GATT Service C). Control plane
    // (commands/events) is wired; the data plane (voice/recording) still needs L2CAP.
    if (!s_tx || !s_tx->start) return ESP_ERR_INVALID_STATE;
    return s_tx->start(s_tx->impl);
}

void agent_link_stop(void) {
    if (s_tx && s_tx->stop) s_tx->stop(s_tx->impl);
    s_state = AGENT_STATE_DISCONNECTED;
}

agent_state_t agent_link_state(void) { return s_state; }

// ── Uplink: events / status (control plane, wired) ───────────────────────────────
esp_err_t agent_link_report_battery(uint8_t percent, bool charging) {
    // Event 0x14 (PowerStatus). payload = {charging_state, level}:
    //   charging_state: 0x00 = not charging, 0x01 = charging, 0x02 = low-battery alert
    //   (below kBatLowThreshold, edge-triggered once).
    // Pushed only on change: the device may call this periodically; the SDK de-duplicates internally.
    if (!s_tx || !s_tx->send_ctrl) return ESP_ERR_INVALID_STATE;
    const int chg = charging ? 1 : 0;
    const bool changed  = (static_cast<int>(percent) != s_bat_pct) || (chg != s_bat_chg);
    const bool low_edge = (percent < kBatLowThreshold) && s_bat_low_armed;
    if (!changed && !low_edge) return ESP_OK;  // no change: do not push

    const uint8_t state = low_edge ? 0x02 : static_cast<uint8_t>(chg);
    uint8_t p[2] = { state, percent };
    auto ev = agentlink::BuildEvent(0x14, p, sizeof(p));
    esp_err_t r = s_tx->send_ctrl(s_tx->impl, ev.data(), ev.size());
    if (r != ESP_OK) return r;  // send failed (not connected / congested): cache not updated, next poll retries so it eventually lands

    // Update the cache / re-arm the alert only after a successful send.
    s_bat_pct = percent;
    s_bat_chg = chg;
    s_bat_low_armed = (percent >= kBatLowThreshold);  // re-arm once back above the threshold; disarm below (already alerted)
    agent_transport_ble_update_battery(percent);      // also update the standard Battery Service (0x2A19) and notify
    ESP_LOGD(TAG, "battery event 0x14: state=%u level=%u%%", state, percent);
    return ESP_OK;
}
esp_err_t agent_link_report_selected_agent(const char* agent_id) {
    (void)agent_id;
    return not_ready("report_selected_agent");  // TODO: 0x16 event (needs index + name_len + name format)
}
esp_err_t agent_link_push_event(agent_event_t type, const uint8_t* data, size_t len) {
    if (!s_tx || !s_tx->send_ctrl) return ESP_ERR_INVALID_STATE;
    if (!(s_tx->is_ready && s_tx->is_ready(s_tx->impl))) return not_ready("push_event");
    // The event_id on the wire is the agent_event_t value: AGENT_EVT_BUTTON/SENSOR/WAKEWORD, or
    // AGENT_EVT_CUSTOM (0x64) for device-private packets — the board owns the payload and the App
    // matches on event_id 0x64 (best-effort, like other events; a dropped frame is not resent).
    auto ev = agentlink::BuildEvent(static_cast<uint8_t>(type), data, len);
    return s_tx->send_ctrl(s_tx->impl, ev.data(), ev.size());
}

// ── Generic I/O: sensors / actuators (see docs/device-io.md; generic channel that does not grow per sensor kind) ──
esp_err_t agent_link_register_io(const agent_link_io_desc_t* desc,
                                 agent_io_actuate_cb_t cb, void* ctx) {
    if (!desc || !desc->id || !desc->kind) return ESP_ERR_INVALID_ARG;
    if (s_io_count >= kMaxIo) {
        ESP_LOGE(TAG, "register_io: registry full (max %d)", kMaxIo);
        return ESP_ERR_NO_MEM;
    }
    s_io[s_io_count] = { desc, cb, ctx };
    ESP_LOGI(TAG, "register_io[%d]: id='%s' kind='%s' dir=%s",
             s_io_count, desc->id, desc->kind, desc->dir == AGENT_IO_OUT ? "out" : "in");
    s_io_count++;
    // The manifest is serialized and sent during the connection handshake (BLE: after the 0xFFC4
    // subscribe; WiFi: after connect), see SendManifest. Downlink actuator commands (0x33) are
    // matched by id to s_io[].cb, see OnCtrlFrame.
    return ESP_OK;
}

// Report one sensor reading: look up the registry for the value type -> validate the length ->
// encode a 0x19 IoReading event -> send_ctrl.
//   payload = [id_len(1)] [id(UTF-8)] [val_type(1)] [value (per type, little-endian)...].
//   Unregistered / non-IN endpoint / length mismatch -> return an error (not sent);
//   not connected -> INVALID_STATE (safely ignored).
esp_err_t agent_link_push_reading(const char* id, const void* value, size_t len) {
    if (!id || !value || len == 0)  return ESP_ERR_INVALID_ARG;
    if (!s_tx || !s_tx->send_ctrl)  return ESP_ERR_INVALID_STATE;
    if (s_tx->is_ready && !s_tx->is_ready(s_tx->impl)) return ESP_ERR_INVALID_STATE;

    const int idx = FindIoIndex(id);
    if (idx < 0 || s_io[idx].desc->dir != AGENT_IO_IN) return ESP_ERR_NOT_FOUND;  // only registered IN endpoints may report
    IoEntry& e = s_io[idx];
    const int want = IoValueSize(e.desc->value);
    if (want >= 0 && static_cast<size_t>(want) != len) {
        ESP_LOGW(TAG, "push_reading '%s': len=%u does not match expected %dB for type",
                 id, static_cast<unsigned>(len), want);
        return ESP_ERR_INVALID_SIZE;
    }
    const size_t id_len = strlen(id);
    if (id_len == 0 || id_len > 255) return ESP_ERR_INVALID_ARG;

    // Cache the latest value (for 0x35 GetReading + on-change dedup) before applying the policy.
    const int64_t now_us = esp_timer_get_time();
    e.last_ts_us = now_us;
    bool same_as_last = false;
    if (len <= sizeof(e.last_val)) {
        same_as_last = e.has_last && e.last_len == len && memcmp(e.last_val, value, len) == 0;
        memcpy(e.last_val, value, len);
        e.last_len = static_cast<uint8_t>(len);
        e.has_last = true;
    }

    // Reporting policy set via 0x36 (default 0 = passthrough).
    switch (e.policy_mode) {
    case IO_POLICY_OFF:
        return ESP_OK;                                         // dropped by policy
    case IO_POLICY_PERIODIC: {
        const int64_t period_us = e.policy_rate_hz ? (1000000 / e.policy_rate_hz) : 0;
        if (period_us && e.last_sent_us && (now_us - e.last_sent_us) < period_us)
            return ESP_OK;                                     // throttled to rate_hz
        break;
    }
    case IO_POLICY_ONCHANGE:
        if (same_as_last) return ESP_OK;                       // unchanged (dedup for fixed-size <=16B values)
        break;
    default:
        break;                                                 // passthrough
    }

    std::vector<uint8_t> p;
    p.reserve(1 + id_len + 1 + len);
    p.push_back(static_cast<uint8_t>(id_len));
    const uint8_t* idb = reinterpret_cast<const uint8_t*>(id);
    p.insert(p.end(), idb, idb + id_len);
    p.push_back(static_cast<uint8_t>(e.desc->value));
    const uint8_t* v = static_cast<const uint8_t*>(value);
    p.insert(p.end(), v, v + len);

    auto ev = agentlink::BuildEvent(0x19, p.data(), p.size());
    const esp_err_t r = s_tx->send_ctrl(s_tx->impl, ev.data(), ev.size());
    if (r == ESP_OK) e.last_sent_us = now_us;
    return r;
}

// ── Generic I/O: notify the Agent that the manifest changed (runtime endpoint add/remove) ──
esp_err_t agent_link_notify_manifest_changed(void) {
    if (!s_tx || !s_tx->send_ctrl) return ESP_ERR_INVALID_STATE;
    ++s_manifest_rev;
    // Event 0x1A ManifestChanged: payload = [rev(4, LE)]. Best-effort; the App re-fetches via 0x34.
    const uint8_t p[4] = { static_cast<uint8_t>(s_manifest_rev),       static_cast<uint8_t>(s_manifest_rev >> 8),
                           static_cast<uint8_t>(s_manifest_rev >> 16), static_cast<uint8_t>(s_manifest_rev >> 24) };
    if (s_tx->is_ready && s_tx->is_ready(s_tx->impl)) {
        auto ev = agentlink::BuildEvent(0x1A, p, sizeof p);
        (void)s_tx->send_ctrl(s_tx->impl, ev.data(), ev.size());
    }
    SendManifest(/*force=*/true);   // re-push the full manifest carrying the new rev
    ESP_LOGI(TAG, "manifest changed -> rev=%u", static_cast<unsigned>(s_manifest_rev));
    return ESP_OK;
}

// ── Data plane: voice uplink (BLE: GATT Notify 0xFFA1, event 0x40 VoiceChunk) ─────
// Feed clean PCM continuously (16kHz/16bit/mono); the first frame lazily opens the session,
// and voice_end is called at the end of an utterance. Slicing/framing/session-number/congestion
// retry all live in the transport backend (transport_ble's voice uploader); the core only
// lazily opens the session and forwards.
esp_err_t agent_link_push_voice(const uint8_t* pcm16, size_t bytes) {
    if (!pcm16 || bytes == 0) return ESP_ERR_INVALID_ARG;
    if (!s_tx || !s_tx->send_stream) return ESP_ERR_INVALID_STATE;
    if (!(s_tx->is_ready && s_tx->is_ready(s_tx->impl))) return not_ready("push_voice");
    if (!s_voice_started) {                       // first frame: open the voice session (allocate session_id + start worker)
        if (s_tx->stream_start) {
            esp_err_t r = s_tx->stream_start(s_tx->impl, AGENT_STREAM_VOICE, nullptr, 0);
            if (r != ESP_OK) return r;
        }
        s_voice_started = true;
    }
    return s_tx->send_stream(s_tx->impl, AGENT_STREAM_VOICE, pcm16, bytes);
}
esp_err_t agent_link_voice_end(void) {
    if (!s_voice_started) return ESP_OK;
    s_voice_started = false;
    if (s_tx && s_tx->stream_end) return s_tx->stream_end(s_tx->impl, AGENT_STREAM_VOICE, /*complete=*/true, nullptr, 0);
    return ESP_OK;
}

// Data plane: real-time ASR audio uplink
// A device to App stream the App transcribes live. agent_link only transports the caller's bytes over the
// record-stream channel: transfer_id + 0x52/0x53 framing + L2CAP chunking/backpressure.
// Requires the App to have opened the L2CAP CoC (PSM 0x0081). One stream at a time; close with asr_end.
esp_err_t agent_link_asr_start(const char* name) {
    if (!s_tx || !s_tx->stream_start) return ESP_ERR_INVALID_STATE;
    if (!(s_tx->is_ready && s_tx->is_ready(s_tx->impl))) return not_ready("asr_start");
    if (s_asr_started) return ESP_OK;  // idempotent: already streaming
    esp_err_t r = s_tx->stream_start(s_tx->impl, AGENT_STREAM_RECORDING,
                                     reinterpret_cast<const uint8_t*>(name), name ? strlen(name) : 0);
    if (r != ESP_OK) return r;
    s_asr_started = true;
    return ESP_OK;
}
esp_err_t agent_link_asr_push(const uint8_t* audio, size_t bytes) {
    if (!audio || bytes == 0) return ESP_ERR_INVALID_ARG;
    if (!s_asr_started) return ESP_ERR_INVALID_STATE;  // call agent_link_asr_start() first
    if (!s_tx || !s_tx->send_stream) return ESP_ERR_INVALID_STATE;
    return s_tx->send_stream(s_tx->impl, AGENT_STREAM_RECORDING, audio, bytes);
}
esp_err_t agent_link_asr_end(bool complete) {
    if (!s_asr_started) return ESP_OK;
    s_asr_started = false;
    if (s_tx && s_tx->stream_end) return s_tx->stream_end(s_tx->impl, AGENT_STREAM_RECORDING, complete, nullptr, 0);
    return ESP_OK;
}

// Data plane: still-image snapshot uplink
// One-shot transparent pipe: pack the 0x54 metadata, open the image stream, queue the bytes, close it.
// The transport owns transfer_id + 0x54/0x55 framing + L2CAP chunking/backpressure and streams async.
esp_err_t agent_link_send_image(const uint8_t* data, size_t bytes,
                                agent_image_format_t fmt, uint16_t w, uint16_t h) {
    if (!data || bytes == 0) return ESP_ERR_INVALID_ARG;
    if (!s_tx || !s_tx->stream_start || !s_tx->send_stream || !s_tx->stream_end) return ESP_ERR_INVALID_STATE;
    if (!(s_tx->is_ready && s_tx->is_ready(s_tx->impl))) return not_ready("send_image");

    // 0x54 metadata: [format(1)][width(2,LE)][height(2,LE)][total_len(4,LE)]. 
    // Sent before any image byte so the App learns the format/size and how many bytes to expect on the L2CAP channel.
    const uint32_t total = static_cast<uint32_t>(bytes);
    const uint8_t meta[9] = {
        static_cast<uint8_t>(fmt),
        static_cast<uint8_t>(w & 0xFF), static_cast<uint8_t>((w >> 8) & 0xFF),
        static_cast<uint8_t>(h & 0xFF), static_cast<uint8_t>((h >> 8) & 0xFF),
        static_cast<uint8_t>(total & 0xFF), static_cast<uint8_t>((total >> 8) & 0xFF),
        static_cast<uint8_t>((total >> 16) & 0xFF), static_cast<uint8_t>((total >> 24) & 0xFF),
    };
    esp_err_t r = s_tx->stream_start(s_tx->impl, AGENT_STREAM_IMAGE, meta, sizeof(meta));
    if (r != ESP_OK) return r;
    r = s_tx->send_stream(s_tx->impl, AGENT_STREAM_IMAGE, data, bytes);
    // Always close so the worker emits 0x55; `complete` reflects whether all bytes were queued.
    esp_err_t e = s_tx->stream_end(s_tx->impl, AGENT_STREAM_IMAGE, r == ESP_OK, nullptr, 0);
    return (r != ESP_OK) ? r : e;
}

// ── Data plane: video (WiFi only; see transport_wifi.cpp)
esp_err_t agent_link_push_video(const uint8_t* frame, size_t bytes, uint32_t pts_ms, bool keyframe) {
    (void)frame; (void)bytes; (void)pts_ms; (void)keyframe; return not_ready("push_video");
}
esp_err_t agent_link_video_end(void) { return not_ready("video_end"); }
