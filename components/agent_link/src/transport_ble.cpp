// agent_link BLE transport backend — advertising + control-plane GATT (NimBLE).
//
// Control plane:
//    Advertise a fixed 128-bit identity UUID (App scan-filters on it)
//    GATT Service C 0xFFC0: 0xFFC1 command channel (WRITE receives commands + NOTIFY returns
//     responses), 0xFFC4 event channel (NOTIFY pushes events).
//    Standard services: 0x180F Battery (0x2A19) + 0x180A Device Information (0x2A29/0x2A24/0x2A26).
//    A write to 0xFFC1 -> s_on_recv up to the core; send_ctrl routes by frame message_type:
//     response (0x02) -> notify 0xFFC1, event (0x03) -> notify 0xFFC4.
// Data plane:
//    Voice uplink (device -> App): Service A 0xFFA0, GATT Notify 0xFFA1 (event 0x40 VoiceChunk).
//    TTS downlink (App -> device): L2CAP CoC (PSM 0x0081) receive, forwarded to the core.
//    ASR/recording uplink (device -> App): L2CAP CoC (PSM 0x0081) send + control events 0x52/0x53.
//    Image snapshot uplink (device -> App): L2CAP CoC (PSM 0x0082) send + control events 0x54/0x55.
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

// Product identity UUID, advertised so the App can scan-filter on it,Fixed constant, identical on every device;
// the device name still distinguishes individual units
// Never change this value: AB883C83-3FCC-4A0F-A951-E18D0C944DA4  (NimBLE wants little-endian byte order)
const ble_uuid128_t s_uuid_identity = BLE_UUID128_INIT(
    0xA4, 0x4D, 0x94, 0x0C, 0x8D, 0xE1, 0x51, 0xA9,
    0x0F, 0x4A, 0xCC, 0x3F, 0x83, 0x3C, 0x88, 0xAB);

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
std::atomic<bool> s_l2cap_stalled{false};  // ble_l2cap_send returned ESTALLED; wait for COC_TX_UNSTALLED before the next send

// L2CAP CoC ( image snapshot uplink, PSM 0x0082)
// A separate PSM from audio so image snapshots and ASR audio can stream concurrently without interleaving on a single byte channel.
constexpr uint16_t kImgPsm = 0x0082;
struct ble_l2cap_chan* s_img_chan = nullptr;
bool s_img_connected = false;
std::atomic<bool> s_img_stalled{false};

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

// ── ASR / recording uplink (L2CAP CoC data plane + control events 0x52/0x53) ──
// device -> App real-time audio: 0x52 StreamStart opens, raw bytes flow over L2CAP CoC, 0x53 StreamEnd closes.
// Transparent pipe: the caller supplies the audio bytes (+ optional 60-byte final_header for 0x53); this
// layer owns transfer_id, framing, MPS-sized chunking with credit backpressure, and honest valid_bytes/status.
constexpr size_t   kRecMaxSdu       = kL2capMps;      // one MPS-sized SDU per send (512 B)
constexpr size_t   kRecQueueHardCap = 128 * 1024;    // backstop: on overflow, truncate the stream (0x53 status=1)
constexpr uint32_t kRecStallWaitMs  = 3000;          // give up (truncate) if the L2CAP tx stays stalled this long
constexpr uint32_t kRecStartDelayMs = 120;           // hold the first L2CAP send so 0x52 StreamStart lands first (App registers the session before audio)
struct RecordingState {
    std::atomic<bool>     active{false};
    std::atomic<bool>     end_req{false};
    std::atomic<bool>     truncated{false};   // overflow / disconnect / send failure -> 0x53 status=1
    std::atomic<bool>     complete{true};     // caller's complete flag, captured at end
    uint32_t              transfer_id = 0;
    std::atomic<uint32_t> sent_bytes{0};      // -> 0x53 valid_bytes
    std::queue<std::vector<uint8_t>> q;
    std::mutex            mtx;
    std::atomic<size_t>   queued_bytes{0};
    std::vector<uint8_t>  final_header;        // 60 B for 0x53 (empty -> all zeros)
    TaskHandle_t          task = nullptr;
};
RecordingState s_rec;
uint32_t       s_rec_counter = 0;

// Image snapshot uplink (L2CAP CoC PSM 0x0082 + control events 0x54/0x55)
// its own PSM and state so it runs concurrently with the ASR audio stream. One image at a time.
// 0x54 ImageStart carries format/size; raw bytes flow over L2CAP; 0x55 ImageEnd reports status/valid_bytes.
constexpr size_t kImgQueueHardCap = 256 * 1024;   // one snapshot fits easily; cap backstops a stuck link
struct ImageState {
    std::atomic<bool>     active{false};
    std::atomic<bool>     end_req{false};
    std::atomic<bool>     truncated{false};   // overflow / disconnect / send failure -> 0x55 status=1
    std::atomic<bool>     complete{true};     // caller's complete flag, captured at end
    uint32_t              transfer_id = 0;
    std::atomic<uint32_t> sent_bytes{0};      // -> 0x55 valid_bytes
    std::queue<std::vector<uint8_t>> q;
    std::mutex            mtx;
    std::atomic<size_t>   queued_bytes{0};
    uint8_t               format = 0;          // agent_image_format_t, carried in 0x54
    uint16_t              width = 0;
    uint16_t              height = 0;
    uint32_t              total_len = 0;       // total image bytes, carried in 0x54
    TaskHandle_t          task = nullptr;
};
ImageState s_img;
uint32_t   s_img_counter = 0;

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

// L2CAP event callback for BOTH CoC servers (audio PSM 0x0081, image PSM 0x0082). The arg passed to ble_l2cap_create_server tells them apart.
// Runs on the NimBLE host task
int L2capEvent(struct ble_l2cap_event* event, void* arg) {
    const bool is_img = (reinterpret_cast<uintptr_t>(arg) == kImgPsm);
    switch (event->type) {
    case BLE_L2CAP_EVENT_COC_ACCEPT: {
        // Critical: allocate the SDU rx buffer + recv_ready first, or the credit handshake never completes. Same for both PSMs.
        struct os_mbuf* rx = os_msys_get_pkthdr(kL2capMps, 0);
        if (!rx) return BLE_HS_ENOMEM;
        int rc = ble_l2cap_recv_ready(event->accept.chan, rx);
        if (rc != 0) { os_mbuf_free_chain(rx); return rc; }
        return 0;
    }
    case BLE_L2CAP_EVENT_COC_CONNECTED:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "L2CAP connect failed (PSM 0x%04X) status=%d",
                     is_img ? kImgPsm : kL2capPsm, event->connect.status);
            return 0;
        }
        if (is_img) { s_img_chan = event->connect.chan;   s_img_connected = true; }
        else        { s_l2cap_chan = event->connect.chan; s_l2cap_connected = true; }
        ESP_LOGI(TAG, "L2CAP CoC connected (PSM 0x%04X)", is_img ? kImgPsm : kL2capPsm);
        return 0;
    case BLE_L2CAP_EVENT_COC_DISCONNECTED:
        if (is_img) {
            s_img_chan = nullptr;
            s_img_connected = false;
            s_img_stalled.store(false, std::memory_order_release);   // unblock a stalled image worker so it can exit
            s_img.truncated.store(true, std::memory_order_release);  // channel gone mid-stream -> truncate (0x55 status=1)
            s_img.end_req.store(true, std::memory_order_release);
        } else {
            s_l2cap_chan = nullptr;
            s_l2cap_connected = false;
            s_l2cap_stalled.store(false, std::memory_order_release); // unblock a stalled recording worker so it can exit
            s_rec.truncated.store(true, std::memory_order_release);  // channel gone mid-stream -> truncate (0x53 status=1 still goes over GATT)
            s_rec.end_req.store(true, std::memory_order_release);
        }
        ESP_LOGI(TAG, "L2CAP CoC disconnected (PSM 0x%04X)", is_img ? kImgPsm : kL2capPsm);
        return 0;
    case BLE_L2CAP_EVENT_COC_DATA_RECEIVED: {
        struct os_mbuf* sdu = event->receive.sdu_rx;
        // The image channel is uplink-only,ignore any downlink bytes on it. The audio channel carries TTS voice downlink. Never block here (host task).
        if (!is_img) {
            const uint16_t total = OS_MBUF_PKTLEN(sdu);
            uint8_t buf[kL2capMps];
            for (uint16_t off = 0; off < total; ) {
                const uint16_t n = static_cast<uint16_t>((total - off < kL2capMps) ? (total - off) : kL2capMps);
                if (os_mbuf_copydata(sdu, off, n, buf) == 0 && s_on_stream) {
                    s_on_stream(AGENT_STREAM_VOICE, buf, n);
                }
                off += n;
            }
        }
        os_mbuf_free_chain(sdu);
        RearmL2capRx(event->receive.chan);  // replenish the rx buffer to keep receiving
        return 0;
    }
    case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
        if (is_img) s_img_stalled.store(false, std::memory_order_release);   // tx credits restored
        else        s_l2cap_stalled.store(false, std::memory_order_release); // recording worker may send again
        return 0;
    default:
        return 0;
    }
}

esp_err_t StartL2capServer() {
    // Audio channel (PSM 0x0081): downlink TTS + uplink ASR/recording
    int rc = ble_l2cap_create_server(kL2capPsm, kL2capMtu, L2capEvent,
                                     reinterpret_cast<void*>(static_cast<uintptr_t>(kL2capPsm)));
    if (rc != 0) { ESP_LOGE(TAG, "l2cap_create_server(audio) rc=%d", rc); return ESP_FAIL; }
    ESP_LOGI(TAG, "L2CAP CoC server on PSM 0x%04X (MTU=%d MPS=%d) — App opens this for audio (TTS/ASR)",
             kL2capPsm, kL2capMtu, kL2capMps);
    // Image channel (PSM 0x0082): uplink still-image snapshots, independent of audio.
    rc = ble_l2cap_create_server(kImgPsm, kL2capMtu, L2capEvent,
                                 reinterpret_cast<void*>(static_cast<uintptr_t>(kImgPsm)));
    if (rc != 0) { ESP_LOGE(TAG, "l2cap_create_server(image) rc=%d", rc); return ESP_FAIL; }
    ESP_LOGI(TAG, "L2CAP CoC server on PSM 0x%04X — App opens this for image snapshots", kImgPsm);
    return ESP_OK;
}

// ASR / recording uplink implementation
// 0x52 StreamStart: payload = transfer_id(4, LE) + filename(N). Notify on 0xFFC4 (DEVICE_EVENT).
bool RecSendStreamStart(uint32_t tid, const char* name, size_t name_len) {
    std::vector<uint8_t> f;
    const uint16_t pl = static_cast<uint16_t>(4 + name_len);
    f.reserve(6 + pl);
    f.push_back(0x01); f.push_back(0x03); f.push_back(0x52); f.push_back(0x00);   // ver, Event, 0x52, seq
    f.push_back(static_cast<uint8_t>(pl & 0xFF)); f.push_back(static_cast<uint8_t>((pl >> 8) & 0xFF));
    f.push_back(tid & 0xFF); f.push_back((tid >> 8) & 0xFF); f.push_back((tid >> 16) & 0xFF); f.push_back((tid >> 24) & 0xFF);
    if (name && name_len) f.insert(f.end(), name, name + name_len);
    // Events get retried (doc §3.3): a lone GATT notify can hit ENOMEM under ACL/msys pressure. Losing the
    // 0x52 means the App never opens the session and drops all the L2CAP audio, so make it best-effort.
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (Notify(s_h_evt, f.data(), f.size()) == ESP_OK) return true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

// 0x53 StreamEnd: payload (69) = transfer_id(4) + status(1) + valid_bytes(4) + final_header(60). Notify on 0xFFC4.
bool RecSendStreamEnd(uint32_t tid, uint8_t status, uint32_t valid_bytes, const uint8_t* hdr60) {
    uint8_t f[6 + 69];
    size_t i = 0;
    f[i++] = 0x01; f[i++] = 0x03; f[i++] = 0x53; f[i++] = 0x00;
    f[i++] = 69; f[i++] = 0x00;                                // payload_len = 69 (LE)
    f[i++] = tid & 0xFF; f[i++] = (tid >> 8) & 0xFF; f[i++] = (tid >> 16) & 0xFF; f[i++] = (tid >> 24) & 0xFF;
    f[i++] = status;
    f[i++] = valid_bytes & 0xFF; f[i++] = (valid_bytes >> 8) & 0xFF; f[i++] = (valid_bytes >> 16) & 0xFF; f[i++] = (valid_bytes >> 24) & 0xFF;
    if (hdr60) memcpy(f + i, hdr60, 60); else memset(f + i, 0, 60);
    i += 60;
    return Notify(s_h_evt, f, i) == ESP_OK;
}

// Send one SDU (<= kRecMaxSdu bytes) over the L2CAP CoC. Returns 0 = sent, 1 = sent but now stalled
// (wait for COC_TX_UNSTALLED), -1 = error (channel down / mbuf pool momentarily empty).
// Ownership: ble_l2cap_send() consumes the mbuf on 0 and ESTALLED; we only free it on a pre-send failure.
int RecL2capSend(const uint8_t* data, size_t len) {
    if (!s_l2cap_connected || !s_l2cap_chan) return -1;
    struct os_mbuf* sdu = os_msys_get_pkthdr(len, 0);
    if (!sdu) return -1;                                        // pool momentarily empty -> caller backs off
    if (os_mbuf_append(sdu, data, len) != 0) { os_mbuf_free_chain(sdu); return -1; }
    int rc = ble_l2cap_send(s_l2cap_chan, sdu);
    if (rc == 0) return 0;
    if (rc == BLE_HS_ESTALLED) { s_l2cap_stalled.store(true, std::memory_order_release); return 1; }
    return -1;                                                 // genuine error: stack owns the sdu
}

// Worker: drain the queue to L2CAP in <=MPS SDUs (credit backpressure via ESTALLED/UNSTALLED),
// then emit 0x53 with the final status + valid_bytes. One stream at a time.
void RecordingTask(void*) {
    ESP_LOGI(TAG, "recording/ASR worker started (transfer_id=%u)", static_cast<unsigned>(s_rec.transfer_id));
    // Head start for 0x52 StreamStart: the App opens the stream session from that GATT event and must have
    // it BEFORE the first L2CAP byte. 0x52 (GATT) and the L2CAP data race on separate channels, and flooding
    // L2CAP immediately starves/drops the 0x52 notify (shared ACL/msys buffers) — then the App discards all
    // audio ("no active session"). Hold the first send; audio pushed meanwhile just queues (< hard cap).
    vTaskDelay(pdMS_TO_TICKS(kRecStartDelayMs));
    while (true) {
        if (!s_connected) { s_rec.truncated.store(true, std::memory_order_release); break; }
        const bool end_req = s_rec.end_req.load(std::memory_order_acquire);
        std::vector<uint8_t> chunk;
        {
            std::lock_guard<std::mutex> lk(s_rec.mtx);
            if (!s_rec.q.empty()) {
                chunk = std::move(s_rec.q.front());
                s_rec.q.pop();
                s_rec.queued_bytes.fetch_sub(chunk.size(), std::memory_order_release);
            }
        }
        if (chunk.empty()) {
            if (end_req) break;                     // drained + end requested -> done
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        bool abort = false;
        for (size_t off = 0; off < chunk.size() && !abort; ) {
            uint32_t waited = 0;                     // credit backpressure: wait while the tx is stalled
            while (s_l2cap_stalled.load(std::memory_order_acquire)) {
                if (!s_connected || !s_l2cap_connected) { abort = true; break; }
                vTaskDelay(pdMS_TO_TICKS(5));
                if ((waited += 5) >= kRecStallWaitMs) { abort = true; break; }  // stuck too long -> truncate
            }
            if (abort) break;
            const size_t n = (chunk.size() - off < kRecMaxSdu) ? (chunk.size() - off) : kRecMaxSdu;
            int r = RecL2capSend(chunk.data() + off, n);
            if (r >= 0) {                            // 0 = sent, 1 = sent then stalled: bytes are on their way
                s_rec.sent_bytes.fetch_add(static_cast<uint32_t>(n), std::memory_order_release);
                off += n;
                if (r == 0) vTaskDelay(pdMS_TO_TICKS(1));   // tiny yield to the host between SDUs
            } else {                                 // pool empty / transient: brief backoff, then retry
                if (!s_connected || !s_l2cap_connected) { abort = true; break; }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        if (abort) { s_rec.truncated.store(true, std::memory_order_release); break; }
    }
    const bool ok = !s_rec.truncated.load(std::memory_order_acquire) && s_rec.complete.load(std::memory_order_acquire);
    const uint8_t status = ok ? 0x00 : 0x01;
    const uint8_t* hdr = (status == 0 && !s_rec.final_header.empty()) ? s_rec.final_header.data() : nullptr;
    RecSendStreamEnd(s_rec.transfer_id, status, s_rec.sent_bytes.load(std::memory_order_acquire), hdr);
    ESP_LOGI(TAG, "recording/ASR stream %u ended (status=%u valid_bytes=%u)",
             static_cast<unsigned>(s_rec.transfer_id), status,
             static_cast<unsigned>(s_rec.sent_bytes.load(std::memory_order_acquire)));
    s_rec.active.store(false, std::memory_order_release);
    s_rec.task = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t RecordingStart(const char* name, size_t name_len) {
    if (s_rec.active.load(std::memory_order_acquire)) return ESP_ERR_INVALID_STATE;  // one stream at a time
    if (!s_connected) return ESP_ERR_INVALID_STATE;
    if (!s_l2cap_connected) {  // "保障通道": the App must have opened the L2CAP CoC before we can push
        ESP_LOGW(TAG, "recording: L2CAP CoC not open — App must connect PSM 0x%04X first", kL2capPsm);
        return ESP_ERR_INVALID_STATE;
    }
    {
        std::lock_guard<std::mutex> lk(s_rec.mtx);
        std::queue<std::vector<uint8_t>> empty;
        std::swap(s_rec.q, empty);
    }
    s_rec.queued_bytes.store(0, std::memory_order_release);
    s_rec.sent_bytes.store(0, std::memory_order_release);
    s_rec.end_req.store(false, std::memory_order_release);
    s_rec.truncated.store(false, std::memory_order_release);
    s_rec.complete.store(true, std::memory_order_release);
    s_rec.final_header.clear();
    s_rec.transfer_id = ++s_rec_counter;
    if (!RecSendStreamStart(s_rec.transfer_id, name, name_len)) {
        ESP_LOGE(TAG, "recording: 0x52 StreamStart notify failed");
        return ESP_FAIL;
    }
    s_rec.active.store(true, std::memory_order_release);
    if (xTaskCreate(RecordingTask, "agentlink_rec", 6144, nullptr, 5, &s_rec.task) != pdPASS) {
        s_rec.active.store(false, std::memory_order_release);
        ESP_LOGE(TAG, "recording: task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "recording/ASR stream %u started (%u-byte name)",
             static_cast<unsigned>(s_rec.transfer_id), static_cast<unsigned>(name_len));
    return ESP_OK;
}

esp_err_t RecordingEnqueue(const uint8_t* data, size_t len) {
    if (!s_rec.active.load(std::memory_order_acquire)) return ESP_ERR_INVALID_STATE;
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    // Once truncated, refuse new bytes: the L2CAP stream is an ordered byte run, so we must not resume
    // after a gap. valid_bytes stays contiguous; the caller should end and (if it wants) start a new stream.
    if (s_rec.truncated.load(std::memory_order_acquire)) return ESP_ERR_NO_MEM;
    std::lock_guard<std::mutex> lk(s_rec.mtx);
    if (s_rec.queued_bytes.load(std::memory_order_acquire) + len > kRecQueueHardCap) {
        s_rec.truncated.store(true, std::memory_order_release);  // link can't keep up -> truncate here (bounded loss)
        s_rec.end_req.store(true, std::memory_order_release);    // drain what's queued (still contiguous), then emit 0x53 status=1
        ESP_LOGW(TAG, "recording queue cap (%uKB) — truncating stream %u",
                 static_cast<unsigned>(kRecQueueHardCap / 1024), static_cast<unsigned>(s_rec.transfer_id));
        return ESP_ERR_NO_MEM;
    }
    s_rec.q.emplace(data, data + len);
    s_rec.queued_bytes.fetch_add(len, std::memory_order_release);
    return ESP_OK;
}

esp_err_t RecordingEnd(bool complete, const uint8_t* final_header, size_t hdr_len) {
    if (!s_rec.active.load(std::memory_order_acquire)) return ESP_OK;
    s_rec.complete.store(complete, std::memory_order_release);
    if (final_header && hdr_len >= 60) s_rec.final_header.assign(final_header, final_header + 60);
    else s_rec.final_header.clear();
    s_rec.end_req.store(true, std::memory_order_release);  // worker drains, emits 0x53, exits
    return ESP_OK;
}

// Image snapshot uplink implementation
// Mirrors the recording path; reuses kRecMaxSdu / kRecStallWaitMs / kRecStartDelayMs (generic L2CAP params).
// 0x54 ImageStart: payload(13) = transfer_id(4) + format(1) + width(2) + height(2) + total_len(4). Notify 0xFFC4.
bool ImgSendStreamStart(uint32_t tid, uint8_t fmt, uint16_t w, uint16_t h, uint32_t total) {
    uint8_t f[6 + 13];
    size_t i = 0;
    f[i++] = 0x01; f[i++] = 0x03; f[i++] = 0x54; f[i++] = 0x00;   // ver, Event, 0x54, seq
    f[i++] = 13; f[i++] = 0x00;                                    // payload_len = 13 (LE)
    f[i++] = tid & 0xFF; f[i++] = (tid >> 8) & 0xFF; f[i++] = (tid >> 16) & 0xFF; f[i++] = (tid >> 24) & 0xFF;
    f[i++] = fmt;
    f[i++] = w & 0xFF; f[i++] = (w >> 8) & 0xFF;
    f[i++] = h & 0xFF; f[i++] = (h >> 8) & 0xFF;
    f[i++] = total & 0xFF; f[i++] = (total >> 8) & 0xFF; f[i++] = (total >> 16) & 0xFF; f[i++] = (total >> 24) & 0xFF;
    // Best-effort retry (like 0x52): losing 0x54 means the App never opens the session and drops the image bytes.
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (Notify(s_h_evt, f, i) == ESP_OK) return true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

// 0x55 ImageEnd: payload(9) = transfer_id(4) + status(1) + valid_bytes(4). Notify 0xFFC4.
bool ImgSendStreamEnd(uint32_t tid, uint8_t status, uint32_t valid_bytes) {
    uint8_t f[6 + 9];
    size_t i = 0;
    f[i++] = 0x01; f[i++] = 0x03; f[i++] = 0x55; f[i++] = 0x00;
    f[i++] = 9; f[i++] = 0x00;                                     // payload_len = 9 (LE)
    f[i++] = tid & 0xFF; f[i++] = (tid >> 8) & 0xFF; f[i++] = (tid >> 16) & 0xFF; f[i++] = (tid >> 24) & 0xFF;
    f[i++] = status;
    f[i++] = valid_bytes & 0xFF; f[i++] = (valid_bytes >> 8) & 0xFF; f[i++] = (valid_bytes >> 16) & 0xFF; f[i++] = (valid_bytes >> 24) & 0xFF;
    return Notify(s_h_evt, f, i) == ESP_OK;
}

// Send one SDU (<= kRecMaxSdu) over the image L2CAP CoC. Returns 0 = sent, 1 = sent but now stalled, -1 = error.
int ImgL2capSend(const uint8_t* data, size_t len) {
    if (!s_img_connected || !s_img_chan) return -1;
    struct os_mbuf* sdu = os_msys_get_pkthdr(len, 0);
    if (!sdu) return -1;
    if (os_mbuf_append(sdu, data, len) != 0) { os_mbuf_free_chain(sdu); return -1; }
    int rc = ble_l2cap_send(s_img_chan, sdu);
    if (rc == 0) return 0;
    if (rc == BLE_HS_ESTALLED) { s_img_stalled.store(true, std::memory_order_release); return 1; }
    return -1;
}

// Worker: drain the image queue to L2CAP in <=MPS SDUs (credit backpressure), then emit 0x55. One image at a time.
void ImageTask(void*) {
    ESP_LOGI(TAG, "image worker started (transfer_id=%u)", static_cast<unsigned>(s_img.transfer_id));
    // Head start for 0x54 ImageStart (same race as ASR 0x52): let the App register the session before bytes arrive.
    vTaskDelay(pdMS_TO_TICKS(kRecStartDelayMs));
    while (true) {
        if (!s_connected) { s_img.truncated.store(true, std::memory_order_release); break; }
        const bool end_req = s_img.end_req.load(std::memory_order_acquire);
        std::vector<uint8_t> chunk;
        {
            std::lock_guard<std::mutex> lk(s_img.mtx);
            if (!s_img.q.empty()) {
                chunk = std::move(s_img.q.front());
                s_img.q.pop();
                s_img.queued_bytes.fetch_sub(chunk.size(), std::memory_order_release);
            }
        }
        if (chunk.empty()) {
            if (end_req) break;                     // drained + end requested -> done
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        bool abort = false;
        for (size_t off = 0; off < chunk.size() && !abort; ) {
            uint32_t waited = 0;                     // credit backpressure: wait while the tx is stalled
            while (s_img_stalled.load(std::memory_order_acquire)) {
                if (!s_connected || !s_img_connected) { abort = true; break; }
                vTaskDelay(pdMS_TO_TICKS(5));
                if ((waited += 5) >= kRecStallWaitMs) { abort = true; break; }  // stuck too long -> truncate
            }
            if (abort) break;
            const size_t n = (chunk.size() - off < kRecMaxSdu) ? (chunk.size() - off) : kRecMaxSdu;
            int r = ImgL2capSend(chunk.data() + off, n);
            if (r >= 0) {                            // 0 = sent, 1 = sent then stalled: bytes are on their way
                s_img.sent_bytes.fetch_add(static_cast<uint32_t>(n), std::memory_order_release);
                off += n;
                if (r == 0) vTaskDelay(pdMS_TO_TICKS(1));   // tiny yield to the host between SDUs
            } else {                                 // pool empty / transient: brief backoff, then retry
                if (!s_connected || !s_img_connected) { abort = true; break; }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        if (abort) { s_img.truncated.store(true, std::memory_order_release); break; }
    }
    const bool ok = !s_img.truncated.load(std::memory_order_acquire) && s_img.complete.load(std::memory_order_acquire);
    const uint8_t status = ok ? 0x00 : 0x01;
    ImgSendStreamEnd(s_img.transfer_id, status, s_img.sent_bytes.load(std::memory_order_acquire));
    ESP_LOGI(TAG, "image stream %u ended (status=%u valid_bytes=%u)",
             static_cast<unsigned>(s_img.transfer_id), status,
             static_cast<unsigned>(s_img.sent_bytes.load(std::memory_order_acquire)));
    s_img.active.store(false, std::memory_order_release);
    s_img.task = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t ImageStart(uint8_t fmt, uint16_t w, uint16_t h, uint32_t total) {
    if (s_img.active.load(std::memory_order_acquire)) return ESP_ERR_INVALID_STATE;  // one image at a time
    if (!s_connected) return ESP_ERR_INVALID_STATE;
    if (!s_img_connected) {  // the App must have opened the image L2CAP CoC before we can push
        ESP_LOGW(TAG, "image: L2CAP CoC not open — App must connect PSM 0x%04X first", kImgPsm);
        return ESP_ERR_INVALID_STATE;
    }
    {
        std::lock_guard<std::mutex> lk(s_img.mtx);
        std::queue<std::vector<uint8_t>> empty;
        std::swap(s_img.q, empty);
    }
    s_img.queued_bytes.store(0, std::memory_order_release);
    s_img.sent_bytes.store(0, std::memory_order_release);
    s_img.end_req.store(false, std::memory_order_release);
    s_img.truncated.store(false, std::memory_order_release);
    s_img.complete.store(true, std::memory_order_release);
    s_img.format = fmt; s_img.width = w; s_img.height = h; s_img.total_len = total;
    s_img.transfer_id = ++s_img_counter;
    if (!ImgSendStreamStart(s_img.transfer_id, fmt, w, h, total)) {
        ESP_LOGE(TAG, "image: 0x54 ImageStart notify failed");
        return ESP_FAIL;
    }
    s_img.active.store(true, std::memory_order_release);
    if (xTaskCreate(ImageTask, "agentlink_img", 6144, nullptr, 5, &s_img.task) != pdPASS) {
        s_img.active.store(false, std::memory_order_release);
        ESP_LOGE(TAG, "image: task create failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ImageEnqueue(const uint8_t* data, size_t len) {
    if (!s_img.active.load(std::memory_order_acquire)) return ESP_ERR_INVALID_STATE;
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    // Once truncated, refuse new bytes: L2CAP is an ordered byte run, so we must not resume after a gap.
    if (s_img.truncated.load(std::memory_order_acquire)) return ESP_ERR_NO_MEM;
    std::lock_guard<std::mutex> lk(s_img.mtx);
    if (s_img.queued_bytes.load(std::memory_order_acquire) + len > kImgQueueHardCap) {
        s_img.truncated.store(true, std::memory_order_release);
        s_img.end_req.store(true, std::memory_order_release);
        ESP_LOGW(TAG, "image queue cap (%uKB) — truncating stream %u",
                 static_cast<unsigned>(kImgQueueHardCap / 1024), static_cast<unsigned>(s_img.transfer_id));
        return ESP_ERR_NO_MEM;
    }
    s_img.q.emplace(data, data + len);
    s_img.queued_bytes.fetch_add(len, std::memory_order_release);
    return ESP_OK;
}

esp_err_t ImageEnd(bool complete) {
    if (!s_img.active.load(std::memory_order_acquire)) return ESP_OK;
    s_img.complete.store(complete, std::memory_order_release);
    s_img.end_req.store(true, std::memory_order_release);  // worker drains, emits 0x55, exits
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
        s_l2cap_stalled.store(false, std::memory_order_release);
        s_voice.end_req.store(true, std::memory_order_release);  // tell the voice worker to wrap up ASAP
        s_rec.truncated.store(true, std::memory_order_release);  // and the recording worker
        s_rec.end_req.store(true, std::memory_order_release);
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
    // Main adv packet: flags + 128-bit identity UUID. The App filters on this UUID
    // No room for the full name here too (31-byte budget: 3 flags + 18 UUID = 21), so the name goes in the scan rsp
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &s_uuid_identity;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields rc=%d", rc); return; }

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = reinterpret_cast<uint8_t*>(s_name);
    rsp.name_len = strlen(s_name);
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) { ESP_LOGE(TAG, "adv_rsp_set_fields rc=%d", rc); return; }

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
esp_err_t ble_stream_start(void* /*impl*/, agent_stream_t type, const uint8_t* meta, size_t meta_len) {
    if (type == AGENT_STREAM_VOICE) return VoiceStart();
    if (type == AGENT_STREAM_RECORDING)                          // ASR / record-stream: 0x52 + L2CAP uplink
        return RecordingStart(reinterpret_cast<const char*>(meta), meta_len);
    if (type == AGENT_STREAM_IMAGE) {                            // image snapshot: 0x54 + L2CAP uplink (PSM 0x0082)
        // meta = [format(1)][width(2,LE)][height(2,LE)][total_len(4,LE)] (packed by agent_link_send_image).
        if (!meta || meta_len < 9) return ESP_ERR_INVALID_ARG;
        const uint8_t  fmt = meta[0];
        const uint16_t w   = static_cast<uint16_t>(meta[1] | (meta[2] << 8));
        const uint16_t h   = static_cast<uint16_t>(meta[3] | (meta[4] << 8));
        const uint32_t total = static_cast<uint32_t>(meta[5]) | (static_cast<uint32_t>(meta[6]) << 8) |
                               (static_cast<uint32_t>(meta[7]) << 16) | (static_cast<uint32_t>(meta[8]) << 24);
        return ImageStart(fmt, w, h, total);
    }
    return ESP_ERR_NOT_SUPPORTED;  // AGENT_STREAM_FILE (L2CAP), VIDEO (WiFi) later
}
esp_err_t ble_send_stream(void* /*impl*/, agent_stream_t type, const uint8_t* data, size_t len) {
    if (type == AGENT_STREAM_VOICE) return VoiceEnqueue(data, len);
    if (type == AGENT_STREAM_RECORDING) return RecordingEnqueue(data, len);
    if (type == AGENT_STREAM_IMAGE) return ImageEnqueue(data, len);
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t ble_stream_end(void* /*impl*/, agent_stream_t type, bool complete, const uint8_t* meta, size_t meta_len) {
    if (type == AGENT_STREAM_VOICE) return VoiceEnd();
    if (type == AGENT_STREAM_RECORDING) return RecordingEnd(complete, meta, meta_len);  // 0x53 (meta = final_header)
    if (type == AGENT_STREAM_IMAGE) return ImageEnd(complete);                          // 0x55
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
