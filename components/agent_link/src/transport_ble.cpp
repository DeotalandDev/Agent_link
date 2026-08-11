// agent_link BLE transport backend — advertising + control-plane GATT (NimBLE).
//
// Control plane:
//   - Advertise the device name; connectable.
//   - GATT Service C 0xFFC0: 0xFFC1 command channel (WRITE receives commands + NOTIFY returns
//     responses), 0xFFC4 event channel (NOTIFY pushes events).
//   - Standard services: 0x180F Battery (0x2A19) + 0x180A Device Information (0x2A29/0x2A24/0x2A26).
//   - A write to 0xFFC1 -> s_on_recv up to the core; send_ctrl routes by frame message_type:
//     response (0x02) -> notify 0xFFC1, event (0x03) -> notify 0xFFC4.
// Data plane:
//   - Voice uplink (device -> App): Service A 0xFFA0, GATT Notify 0xFFA1 (event 0x40 VoiceChunk).
//   - TTS downlink (App -> device): L2CAP CoC (PSM 0x0081) receive, forwarded to the core.
// Not done: L2CAP uplink (recording/file/OTA), Service B, encryption.
#include "agent_link_transport.h"

#include <cstring>
#include <queue>
#include <vector>
#include <mutex>
#include <atomic>

#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"
#include "host/ble_l2cap.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"      // os_msys_get_pkthdr / os_mbuf_copydata

extern "C" void ble_store_config_init(void);

namespace {
constexpr const char* TAG = "agent_link.ble";

// ── Service A (intercom / realtime voice): device -> App PCM voice stream ──
constexpr uint16_t kSvcVoice      = 0xFFA0;
constexpr uint16_t kChrVoiceDown  = 0xFFA1;  // NOTIFY: device -> App PCM (event 0x40 VoiceChunk)
// ── Service C (device control) ──
constexpr uint16_t kSvcDeviceControl = 0xFFC0;
constexpr uint16_t kChrCmdRequest    = 0xFFC1;  // WRITE receives commands + NOTIFY returns responses
constexpr uint16_t kChrDeviceEvent   = 0xFFC4;  // NOTIFY pushes events
// Note: 0xFFA2 (PTT) / 0xFFC2 (state) / 0xFFC3 (config) were unused placeholders and have been removed.
// ── Standard services ──
constexpr uint16_t kSvcBattery      = 0x180F;   // Battery Service
constexpr uint16_t kChrBatteryLevel = 0x2A19;   // READ + NOTIFY, 1-byte battery level 0-100
constexpr uint16_t kSvcDeviceInfo   = 0x180A;   // Device Information Service
constexpr uint16_t kChrManufacturer = 0x2A29;   // READ string
constexpr uint16_t kChrModelNumber  = 0x2A24;   // READ string
constexpr uint16_t kChrFirmwareRev  = 0x2A26;   // READ string

// Static UUID objects (avoid BLE_UUID16_DECLARE compound literals in static initialization).
const ble_uuid16_t s_uuid_voice_svc = BLE_UUID16_INIT(kSvcVoice);
const ble_uuid16_t s_uuid_voice     = BLE_UUID16_INIT(kChrVoiceDown);
const ble_uuid16_t s_uuid_svc   = BLE_UUID16_INIT(kSvcDeviceControl);
const ble_uuid16_t s_uuid_cmd   = BLE_UUID16_INIT(kChrCmdRequest);
const ble_uuid16_t s_uuid_evt   = BLE_UUID16_INIT(kChrDeviceEvent);
const ble_uuid16_t s_uuid_svc_batt  = BLE_UUID16_INIT(kSvcBattery);
const ble_uuid16_t s_uuid_batt_lvl  = BLE_UUID16_INIT(kChrBatteryLevel);
const ble_uuid16_t s_uuid_svc_dis   = BLE_UUID16_INIT(kSvcDeviceInfo);
const ble_uuid16_t s_uuid_mfr       = BLE_UUID16_INIT(kChrManufacturer);
const ble_uuid16_t s_uuid_model     = BLE_UUID16_INIT(kChrModelNumber);
const ble_uuid16_t s_uuid_fwrev     = BLE_UUID16_INIT(kChrFirmwareRev);

char     s_name[31] = "AgentLink";
uint8_t  s_own_addr_type = 0;
bool     s_connected = false;
uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// GATT characteristic value handles (filled in by NimBLE at registration; used for notify).
uint16_t s_h_cmd   = 0; // 0xFFC1
uint16_t s_h_evt   = 0; // 0xFFC4
uint16_t s_h_voice = 0; // 0xFFA1 (voice upload)
uint16_t s_h_batt  = 0; // 0x2A19 (battery, notify)

// Standard-service values (returned by the read callback; battery notifies 0x2A19 on change).
uint8_t s_batt_level = 0;              // current battery level 0-100
char    s_di_mfr[32]   = "Deotaland";  // 0x2A29 manufacturer
char    s_di_model[32] = "AgentLink";  // 0x2A24 model (overridden by init with the device name)
char    s_di_fw[24]    = "1.0.0";      // 0x2A26 firmware revision

// Transport -> core uplink callbacks (installed by the core via set_recv/set_conn/set_stream_recv).
void (*s_on_recv)(const uint8_t*, size_t) = nullptr;
void (*s_on_conn)(bool) = nullptr;
void (*s_on_stream)(agent_stream_t, const uint8_t*, size_t) = nullptr;  // data-plane receive (L2CAP)
void (*s_on_ready)(void) = nullptr;  // peer subscribed to 0xFFC4 -> notify reachable (core sends the manifest)

// ── L2CAP CoC (data plane: receive App downlink TTS voice, PSM 0x0081) ──
constexpr uint16_t kL2capPsm = 0x0081;  // PSM of the downlink voice channel
constexpr uint16_t kL2capMtu = 4096;    // our_coc_mtu (receive)
constexpr uint16_t kL2capMps = 512;     // per-packet size (the SDU rx buffer is allocated to this)
struct ble_l2cap_chan* s_l2cap_chan = nullptr;
bool s_l2cap_connected = false;

void StartAdvertising();

// GATT access callback: command writes + standard-service reads.
int GattAccess(uint16_t /*conn*/, uint16_t /*attr*/, struct ble_gatt_access_ctxt* ctxt, void* /*arg*/) {
    const ble_uuid_t* uuid = ctxt->chr->uuid;
    if (uuid->type != BLE_UUID_TYPE_16) return BLE_ATT_ERR_UNLIKELY;
    const uint16_t chr = BLE_UUID16(uuid)->value;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        // Only the command channel 0xFFC1 takes writes -> copy the bytes out -> up to the core to parse.
        if (chr == kChrCmdRequest && ctxt->om) {
            uint8_t buf[512];
            uint16_t n = 0;
            if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &n) == 0 && n > 0 && s_on_recv) {
                s_on_recv(buf, n);
            }
        }
        return 0;
    }
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        // Standard-service read: return battery level / device-info strings.
        const void* val = nullptr;
        uint16_t vlen = 0;
        switch (chr) {
        case kChrBatteryLevel: val = &s_batt_level; vlen = 1; break;
        case kChrManufacturer: val = s_di_mfr;   vlen = static_cast<uint16_t>(strlen(s_di_mfr));   break;
        case kChrModelNumber:  val = s_di_model; vlen = static_cast<uint16_t>(strlen(s_di_model)); break;
        case kChrFirmwareRev:  val = s_di_fw;    vlen = static_cast<uint16_t>(strlen(s_di_fw));    break;
        default: return 0;  // other reads: empty
        }
        return os_mbuf_append(ctxt->om, val, vlen) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    default:
        return 0;
    }
}

// Service C (device control): 0xFFC1 command (WRITE+NOTIFY) + 0xFFC4 event (NOTIFY). Placeholders removed.
const struct ble_gatt_chr_def s_chrs[] = {
    { .uuid = &s_uuid_cmd.u, .access_cb = GattAccess,
      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_h_cmd },
    { .uuid = &s_uuid_evt.u, .access_cb = GattAccess,
      .flags = BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_h_evt },
    { 0 },
};
// Service A (intercom / realtime voice): only 0xFFA1 VOICE_DOWN (NOTIFY). PTT placeholder removed.
const struct ble_gatt_chr_def s_chrs_voice[] = {
    { .uuid = &s_uuid_voice.u, .access_cb = GattAccess,
      .flags = BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_h_voice },
    { 0 },
};
// Standard service 0x180F Battery: 0x2A19 level (READ+NOTIFY).
const struct ble_gatt_chr_def s_chrs_batt[] = {
    { .uuid = &s_uuid_batt_lvl.u, .access_cb = GattAccess,
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_h_batt },
    { 0 },
};
// Standard service 0x180A Device Information: manufacturer / model / firmware (all READ).
const struct ble_gatt_chr_def s_chrs_dis[] = {
    { .uuid = &s_uuid_mfr.u,   .access_cb = GattAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &s_uuid_model.u, .access_cb = GattAccess, .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &s_uuid_fwrev.u, .access_cb = GattAccess, .flags = BLE_GATT_CHR_F_READ },
    { 0 },
};
const struct ble_gatt_svc_def s_svcs[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &s_uuid_voice_svc.u, .characteristics = s_chrs_voice },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &s_uuid_svc.u,       .characteristics = s_chrs },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &s_uuid_svc_batt.u,  .characteristics = s_chrs_batt },
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &s_uuid_svc_dis.u,   .characteristics = s_chrs_dis },
    { 0 },
};

esp_err_t RegisterGatt() {
    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "count_cfg rc=%d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "add_svcs rc=%d", rc); return ESP_FAIL; }
    return ESP_OK;
}

esp_err_t Notify(uint16_t handle, const uint8_t* data, size_t len) {
    if (!s_connected || handle == 0) return ESP_ERR_INVALID_STATE;
    struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return ESP_ERR_NO_MEM;
    int rc = ble_gatts_notify_custom(s_conn_handle, handle, om);  // on failure NimBLE has already freed om
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

// ── Voice upload (0x40 VoiceChunk, Notify 0xFFA1) ──
// Integrity-first queue + worker task + MTU-aware slicing.
// Frame overhead = 6 (common header) + session(4) + sequence(4) + flags(1) = 15.
constexpr size_t   kVoiceOverhead      = 15;
constexpr size_t   kMaxFrameSingleMbuf = 220;  // keep a frame in a single mbuf, avoiding chain-alloc failures on high-rate notify
constexpr size_t   kMaxPcmSingleMbuf   = kMaxFrameSingleMbuf - kVoiceOverhead;  // 205
constexpr size_t   kFallbackPcmBytes   = 80;   // conservative slice when the MTU is abnormal
constexpr size_t   kVoiceQueueHardCap  = 96 * 1024;  // hard cap ~3s @ 32KB/s: when the link is bad, drop new frames to keep the head
constexpr uint32_t kVoiceMaxRetries    = 300;  // per-slice congestion retry limit

struct VoiceState {
    std::atomic<bool>     active{false};
    std::atomic<bool>     end_req{false};
    uint32_t              session_id = 0;
    std::atomic<uint32_t> sequence{0};
    std::queue<std::vector<uint8_t>> q;
    std::mutex            mtx;
    std::atomic<size_t>   queued_bytes{0};
    TaskHandle_t          task = nullptr;
};
VoiceState s_voice;
uint32_t   s_voice_counter = 0;

// Assemble one 0x40 VoiceChunk frame and notify it on 0xFFA1. len <= kMaxPcmSingleMbuf.
bool VoiceSendChunk(uint32_t sequence, const uint8_t* data, size_t len) {
    uint8_t f[kMaxFrameSingleMbuf];
    size_t i = 0;
    f[i++] = 0x01;                          // version
    f[i++] = 0x03;                          // message_type = Event
    f[i++] = 0x40;                          // command_id = VoiceChunk
    f[i++] = 0x00;                          // sequence (unused for Event)
    const uint16_t pl = static_cast<uint16_t>(4 + 4 + 1 + len);
    f[i++] = pl & 0xFF;
    f[i++] = (pl >> 8) & 0xFF;
    f[i++] = s_voice.session_id & 0xFF;     // session_id (little-endian)
    f[i++] = (s_voice.session_id >> 8) & 0xFF;
    f[i++] = (s_voice.session_id >> 16) & 0xFF;
    f[i++] = (s_voice.session_id >> 24) & 0xFF;
    f[i++] = sequence & 0xFF;               // sequence (little-endian, monotonic across slices)
    f[i++] = (sequence >> 8) & 0xFF;
    f[i++] = (sequence >> 16) & 0xFF;
    f[i++] = (sequence >> 24) & 0xFF;
    f[i++] = 0x00;                          // flags (0 = normal frame)
    memcpy(f + i, data, len);
    i += len;
    return Notify(s_h_voice, f, i) == ESP_OK;
}

void VoiceTask(void*) {
    ESP_LOGI(TAG, "voice task started (session=%u)", static_cast<unsigned>(s_voice.session_id));
    while (true) {
        if (!s_connected) { ESP_LOGW(TAG, "voice: link down — stop"); break; }
        const bool end_req = s_voice.end_req.load(std::memory_order_acquire);

        std::vector<uint8_t> chunk;
        {
            std::lock_guard<std::mutex> lk(s_voice.mtx);
            if (!s_voice.q.empty()) {
                chunk = std::move(s_voice.q.front());
                s_voice.q.pop();
                s_voice.queued_bytes.fetch_sub(chunk.size(), std::memory_order_release);
            }
        }
        if (chunk.empty()) {
            if (end_req) { ESP_LOGI(TAG, "voice: drained — done"); break; }
            vTaskDelay(pdMS_TO_TICKS(10));  // empty queue: yield at least 1 tick (FREERTOS_HZ=100)
            continue;
        }

        // MTU-aware slicing: per-notify PCM = ATT_MTU - 3 - 15, clamped to a single mbuf (205) and even-aligned.
        const uint16_t mtu = (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) ? ble_att_mtu(s_conn_handle) : 0;
        size_t max_pcm = (static_cast<size_t>(mtu) > 3 + kVoiceOverhead + 2)
                             ? (static_cast<size_t>(mtu) - 3 - kVoiceOverhead)
                             : kFallbackPcmBytes;
        if (max_pcm > kMaxPcmSingleMbuf) max_pcm = kMaxPcmSingleMbuf;
        max_pcm &= ~static_cast<size_t>(1);           // even-aligned (int16 samples)
        if (max_pcm == 0) max_pcm = 2;

        for (size_t off = 0; off < chunk.size(); ) {
            const size_t remaining = chunk.size() - off;
            const size_t n = (remaining < max_pcm) ? remaining : max_pcm;
            const uint32_t seq = s_voice.sequence.fetch_add(1, std::memory_order_release);

            bool sent = false;
            for (uint32_t retry = 0; retry < kVoiceMaxRetries; retry++) {
                if (!s_connected) break;
                if (VoiceSendChunk(seq, chunk.data() + off, n)) { sent = true; break; }
                // congestion backoff: 1ms x5 -> 5ms x15 -> 10ms (a failed notify signals mbuf-pool pressure).
                const uint32_t d = (retry < 5) ? 1 : (retry < 20) ? 5 : 10;
                vTaskDelay(pdMS_TO_TICKS(d));
            }
            if (!sent) { ESP_LOGE(TAG, "voice: slice send failed — stop"); s_voice.end_req.store(true); break; }
            off += n;
            vTaskDelay(pdMS_TO_TICKS(1));  // small yield per slice to give the BLE host a chance
        }
    }
    s_voice.active.store(false, std::memory_order_release);
    s_voice.task = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t VoiceStart() {
    if (s_voice.active.load(std::memory_order_acquire)) return ESP_OK;  // already in a session
    if (!s_connected) return ESP_ERR_INVALID_STATE;
    {   // reset queue + counters
        std::lock_guard<std::mutex> lk(s_voice.mtx);
        std::queue<std::vector<uint8_t>> empty;
        std::swap(s_voice.q, empty);
    }
    s_voice.queued_bytes.store(0, std::memory_order_release);
    s_voice.sequence.store(0, std::memory_order_release);
    s_voice.end_req.store(false, std::memory_order_release);
    s_voice.session_id = ++s_voice_counter;
    s_voice.active.store(true, std::memory_order_release);
    // Use the internal stack (NimBLE's notify call chain is deep, so leave headroom; do not assume PSRAM, for portability). Priority 6.
    if (xTaskCreate(VoiceTask, "agentlink_voice", 6144, nullptr, 6, &s_voice.task) != pdPASS) {
        s_voice.active.store(false, std::memory_order_release);
        ESP_LOGE(TAG, "voice: task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "voice session %u started (mtu=%u)",
             static_cast<unsigned>(s_voice.session_id),
             static_cast<unsigned>(s_conn_handle != BLE_HS_CONN_HANDLE_NONE ? ble_att_mtu(s_conn_handle) : 0));
    return ESP_OK;
}

esp_err_t VoiceEnqueue(const uint8_t* data, size_t len) {
    if (!s_voice.active.load(std::memory_order_acquire)) return ESP_ERR_INVALID_STATE;
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lk(s_voice.mtx);
    // Integrity-first, with a hard cap as a backstop: when the link is bad, drop new frames to keep the head (the App at least gets the start of the voice).
    if (s_voice.queued_bytes.load(std::memory_order_acquire) + len > kVoiceQueueHardCap) {
        ESP_LOGW(TAG, "voice queue hard cap (%uKB) — drop %uB (link stalled)",
                 static_cast<unsigned>(kVoiceQueueHardCap / 1024), static_cast<unsigned>(len));
        return ESP_ERR_NO_MEM;
    }
    s_voice.q.emplace(data, data + len);
    s_voice.queued_bytes.fetch_add(len, std::memory_order_release);
    return ESP_OK;
}

esp_err_t VoiceEnd() {
    if (!s_voice.active.load(std::memory_order_acquire)) return ESP_OK;
    s_voice.end_req.store(true, std::memory_order_release);  // the worker drains the rest, then exits on its own
    return ESP_OK;
}

// ── L2CAP CoC receive (downlink TTS voice, PSM 0x0081) ──
// Replenishing the rx buffer grants the peer credit and keeps the channel able to receive.
bool RearmL2capRx(struct ble_l2cap_chan* chan) {
    struct os_mbuf* rx = os_msys_get_pkthdr(kL2capMps, 0);
    if (!rx) return false;
    if (ble_l2cap_recv_ready(chan, rx) != 0) { os_mbuf_free_chain(rx); return false; }
    return true;
}

// L2CAP event callback (runs on the NimBLE host task; do not block).
int L2capEvent(struct ble_l2cap_event* event, void* /*arg*/) {
    switch (event->type) {
    case BLE_L2CAP_EVENT_COC_ACCEPT: {
        // Critical: allocate the SDU rx buffer + recv_ready first, or the credit handshake never
        // completes (NimBLE rejects with "No Resources"; iOS reports a didOpen error).
        struct os_mbuf* rx = os_msys_get_pkthdr(kL2capMps, 0);
        if (!rx) return BLE_HS_ENOMEM;
        int rc = ble_l2cap_recv_ready(event->accept.chan, rx);
        if (rc != 0) { os_mbuf_free_chain(rx); return rc; }
        return 0;
    }
    case BLE_L2CAP_EVENT_COC_CONNECTED:
        if (event->connect.status == 0) {
            s_l2cap_chan = event->connect.chan;
            s_l2cap_connected = true;
            ESP_LOGI(TAG, "L2CAP CoC connected (PSM 0x%04X)", kL2capPsm);
        } else {
            ESP_LOGW(TAG, "L2CAP connect failed status=%d", event->connect.status);
        }
        return 0;
    case BLE_L2CAP_EVENT_COC_DISCONNECTED:
        s_l2cap_chan = nullptr;
        s_l2cap_connected = false;
        ESP_LOGI(TAG, "L2CAP CoC disconnected");
        return 0;
    case BLE_L2CAP_EVENT_COC_DATA_RECEIVED: {
        struct os_mbuf* sdu = event->receive.sdu_rx;
        const uint16_t total = OS_MBUF_PKTLEN(sdu);
        // Copy out in chunks and forward to the core. The only downlink data plane right now is TTS voice -> AGENT_STREAM_VOICE.
        // Do not block here (NimBLE host task) — the core/board should enqueue quickly and play in another task.
        uint8_t buf[kL2capMps];
        for (uint16_t off = 0; off < total; ) {
            const uint16_t n = static_cast<uint16_t>((total - off < kL2capMps) ? (total - off) : kL2capMps);
            if (os_mbuf_copydata(sdu, off, n, buf) == 0 && s_on_stream) {
                s_on_stream(AGENT_STREAM_VOICE, buf, n);
            }
            off += n;
        }
        os_mbuf_free_chain(sdu);            // NimBLE does not auto-free; must free explicitly
        RearmL2capRx(event->receive.chan);  // replenish the rx buffer to keep receiving
        return 0;
    }
    case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
        return 0;  // send side (recording/file uplink) to be used later
    default:
        return 0;
    }
}

esp_err_t StartL2capServer() {
    int rc = ble_l2cap_create_server(kL2capPsm, kL2capMtu, L2capEvent, nullptr);
    if (rc != 0) { ESP_LOGE(TAG, "l2cap_create_server rc=%d", rc); return ESP_FAIL; }
    ESP_LOGI(TAG, "L2CAP CoC server on PSM 0x%04X (MTU=%d MPS=%d) — App must explicitly open this PSM after connecting",
             kL2capPsm, kL2capMtu, kL2capMps);
    return ESP_OK;
}

// GAP events: connect / disconnect / advertising complete.
int GapEvent(struct ble_gap_event* event, void* /*arg*/) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connected = true;
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected (handle=%d)", s_conn_handle);
            if (s_on_conn) s_on_conn(true);
        } else {
            ESP_LOGW(TAG, "connect failed (status=%d) — re-advertising", event->connect.status);
            StartAdvertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        s_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_l2cap_connected = false; s_l2cap_chan = nullptr;  // L2CAP goes down with the ACL
        s_voice.end_req.store(true, std::memory_order_release);  // tell the voice worker to wrap up ASAP
        ESP_LOGI(TAG, "disconnected (reason=%d) — re-advertising", event->disconnect.reason);
        if (s_on_conn) s_on_conn(false);
        StartAdvertising();
        return 0;
    case BLE_GAP_EVENT_ENC_CHANGE:
        // Link encryption (re)established: either a fresh pairing just completed, or a bonded peer
        // reconnected and the stored LTK was restored. status==0 => encrypted, no re-pairing needed.
        ESP_LOGI(TAG, "encryption change: status=%d%s", event->enc_change.status,
                 event->enc_change.status == 0 ? " (encrypted; bond in use)" : " (failed)");
        return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // The peer initiates pairing even though we still hold a bond for it (it wiped its keys, or
        // the user chose "forget this device"). Delete our stale bond and let the new pairing go
        // through — otherwise the stack would drop the connection.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_SUBSCRIBE:
        // Peer subscribed to the event channel 0xFFC4 (CCCD write enables notify) -> notifications
        // are delivered from now on -> tell the core it is "notify-ready" (the core sends the I/O
        // manifest from here, see agent_link.cpp OnLinkReady).
        if (event->subscribe.attr_handle == s_h_evt && event->subscribe.cur_notify && s_on_ready) {
            ESP_LOGI(TAG, "peer subscribed 0xFFC4 (event channel) — notify-ready");
            s_on_ready();
        }
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        StartAdvertising();
        return 0;
    default:
        return 0;
    }
}

void StartAdvertising() {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<uint8_t*>(s_name);
    fields.name_len = strlen(s_name);
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields rc=%d", rc); return; }

    struct ble_gap_adv_params adv;
    memset(&adv, 0, sizeof(adv));
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;   // connectable
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;   // generally discoverable
    rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER, &adv, GapEvent, nullptr);
    if (rc != 0) { ESP_LOGE(TAG, "adv_start rc=%d", rc); return; }
    ESP_LOGI(TAG, "advertising as '%s'", s_name);
}

void OnSync() {
    if (ble_hs_util_ensure_addr(0) != 0) { ESP_LOGE(TAG, "ensure_addr failed"); return; }
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) { ESP_LOGE(TAG, "infer_auto failed"); return; }
    (void)StartL2capServer();  // once the stack is ready, create the L2CAP CoC server (downlink TTS voice channel)
    StartAdvertising();
}

void OnReset(int reason) { ESP_LOGW(TAG, "nimble reset; reason=%d", reason); }

void HostTask(void* /*param*/) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t EnsureNvs() {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        (void)nvs_flash_erase();
        r = nvs_flash_init();
    }
    return r;
}

// ── agent_transport_t interface implementation ──
esp_err_t ble_start(void* /*impl*/) {
    esp_err_t r = EnsureNvs();
    if (r != ESP_OK) { ESP_LOGE(TAG, "nvs init: %s", esp_err_to_name(r)); return r; }

    r = nimble_port_init();
    if (r != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(r)); return r; }

    ble_hs_cfg.sync_cb  = OnSync;
    ble_hs_cfg.reset_cb = OnReset;

    ble_hs_cfg.sm_bonding = 1;                          // keep the keys after pairing
    ble_hs_cfg.sm_sc      = 1;                          // LE Secure Connections
    ble_hs_cfg.sm_io_cap  = BLE_HS_IO_NO_INPUT_OUTPUT;

    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    if (RegisterGatt() != ESP_OK) return ESP_FAIL;
    ble_svc_gap_device_name_set(s_name);

    ble_store_config_init();  // install the key store
    nimble_port_freertos_init(HostTask);
    ESP_LOGI(TAG, "BLE started — Service C 0xFFC0 registered; advertising on sync");
    return ESP_OK;
}

void ble_stop(void* /*impl*/) {
    (void)ble_gap_adv_stop();
    s_connected = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
}

// Control-plane send: route by frame message_type. response (0x02) -> 0xFFC1; event (0x03) -> 0xFFC4.
esp_err_t ble_send_ctrl(void* /*impl*/, const uint8_t* frame, size_t len) {
    if (!frame || len < 2) return ESP_ERR_INVALID_ARG;
    const uint8_t msg_type = frame[1] & 0x7F;
    const uint16_t handle = (msg_type == 0x03 /*Event*/) ? s_h_evt : s_h_cmd;
    return Notify(handle, frame, len);
}

// Data plane: voice over GATT Notify 0xFFA1 (implemented); recording/file over L2CAP, video WiFi-only (to do).
esp_err_t ble_stream_start(void* /*impl*/, agent_stream_t type, const uint8_t* /*meta*/, size_t /*meta_len*/) {
    if (type == AGENT_STREAM_VOICE) return VoiceStart();
    return ESP_ERR_NOT_SUPPORTED;  // AGENT_STREAM_RECORDING/FILE (L2CAP), VIDEO (WiFi) later
}
esp_err_t ble_send_stream(void* /*impl*/, agent_stream_t type, const uint8_t* data, size_t len) {
    if (type == AGENT_STREAM_VOICE) return VoiceEnqueue(data, len);
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t ble_stream_end(void* /*impl*/, agent_stream_t type, bool /*complete*/) {
    if (type == AGENT_STREAM_VOICE) return VoiceEnd();
    return ESP_ERR_NOT_SUPPORTED;
}
bool ble_is_ready(void* /*impl*/) { return s_connected; }

agent_transport_t s_ble = {
    ble_start, ble_stop, ble_send_ctrl,
    ble_stream_start, ble_send_stream, ble_stream_end,
    ble_is_ready, nullptr,
};
}  // namespace

extern "C" agent_transport_t* agent_transport_ble(void) { return &s_ble; }

extern "C" void agent_transport_ble_set_name(const char* name) {
    if (name && *name) {
        strncpy(s_name, name, sizeof(s_name) - 1);
        s_name[sizeof(s_name) - 1] = '\0';
    }
}
extern "C" void agent_transport_ble_set_recv(void (*cb)(const uint8_t*, size_t)) { s_on_recv = cb; }
extern "C" void agent_transport_ble_set_conn(void (*cb)(bool)) { s_on_conn = cb; }
extern "C" void agent_transport_ble_set_stream_recv(void (*cb)(agent_stream_t, const uint8_t*, size_t)) {
    s_on_stream = cb;
}
extern "C" void agent_transport_ble_set_ready(void (*cb)(void)) { s_on_ready = cb; }

// Current negotiated ATT MTU (0 if not connected). Used by the core to adapt manifest fragment size.
extern "C" uint16_t agent_transport_ble_att_mtu(void) {
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return 0;
    return ble_att_mtu(s_conn_handle);
}

// Device info (the three 0x180A standard-service strings). Called once at init; NULL/empty keeps defaults.
extern "C" void agent_transport_ble_set_device_info(const char* mfr, const char* model, const char* fw) {
    if (mfr && *mfr)   { strncpy(s_di_mfr, mfr, sizeof(s_di_mfr) - 1);     s_di_mfr[sizeof(s_di_mfr) - 1] = '\0'; }
    if (model && *model){ strncpy(s_di_model, model, sizeof(s_di_model) - 1); s_di_model[sizeof(s_di_model) - 1] = '\0'; }
    if (fw && *fw)     { strncpy(s_di_fw, fw, sizeof(s_di_fw) - 1);        s_di_fw[sizeof(s_di_fw) - 1] = '\0'; }
}

// Update battery (standard service 0x180F). Store the value for reads; notify 0x2A19 if connected. Called by the core when the level changes.
extern "C" void agent_transport_ble_update_battery(uint8_t percent) {
    s_batt_level = (percent > 100) ? 100 : percent;
    if (s_connected && s_h_batt) (void)Notify(s_h_batt, &s_batt_level, 1);
}
