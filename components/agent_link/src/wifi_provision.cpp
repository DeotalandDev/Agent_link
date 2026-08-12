// agent_link WiFi provisioning — SoftAP captive portal (配网).
//
// Flow: no stored credentials -> the transport brings up an open SoftAP and calls
// al_wifi_prov_start(). This module then runs:
//   - a tiny DNS server that answers every A query with the SoftAP IP, so the phone's
//     captive-portal probe resolves to us and the "sign in to network" sheet pops open;
//   - an HTTP server serving the config page (portal/portal.html), accepting the submitted
//     SSID/password (POST /provision), and reporting join progress (GET /status).
//
// The web UI lives in editable files under portal/ (embedded via CMakeLists EMBED_TXTFILES), so
// downstream users can restyle/translate the pages without touching C. The firmware only injects the
// scanned network list at the {{SSIDS}} marker; language switching is pure CSS inside the page.
#include "wifi_provision.h"

#include <cstring>
#include <cstdio>
#include <string>
#include <mutex>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"

// Web assets embedded at build time (CMakeLists EMBED_TXTFILES). Edit portal/*.html to restyle or
// translate the pages — no C changes needed. EMBED_TXTFILES NUL-terminates each blob.
extern const char portal_html_start[]     asm("_binary_portal_html_start");
extern const char connecting_html_start[] asm("_binary_connecting_html_start");

namespace {
constexpr const char* TAG = "agent_link.prov";

httpd_handle_t     s_httpd      = nullptr;
TaskHandle_t       s_dns_task   = nullptr;
volatile bool      s_dns_run    = false;
al_prov_creds_cb_t s_on_creds   = nullptr;
esp_ip4_addr_t     s_ap_ip      = { .addr = 0 };   // SoftAP gateway IP, network byte order
char               s_ap_ip_str[16] = "192.168.4.1";

// Progress the page polls at /status. ssid is captured at submit; ip/reason set by the transport.
struct ProvStatus {
    al_prov_status_t st = AL_PROV_IDLE;
    char ip[16]    = {0};
    char ssid[33]  = {0};
    char reason[16] = {0};   // short code (badpass/notfound/fail); localized by the page
};
ProvStatus s_status;
std::mutex s_status_mtx;

// Cached WiFi scan (populated once when provisioning starts; served to the page + /scan).
// We never scan live from an HTTP handler: on a single radio a scan drops the SoftAP client that
// triggered it, killing the request. Scan once up-front (before anyone connects) and cache it.
std::string  s_scan_json    = "[]";   // GET /scan (JSON)
std::string  s_scan_options;          // <option> list injected into the page (GET /)
std::mutex   s_scan_mtx;
TaskHandle_t s_scan_task     = nullptr;

const char* StatusName(al_prov_status_t s) {
    switch (s) {
    case AL_PROV_CONNECTING: return "connecting";
    case AL_PROV_CONNECTED:  return "connected";
    case AL_PROV_FAILED:     return "failed";
    default:                 return "idle";
    }
}

// ── Small helpers ──────────────────────────────────────────────────────────────────────────
// Percent-decode an application/x-www-form-urlencoded value ('+' -> space, %XX -> byte).
void UrlDecode(char* dst, size_t dst_sz, const char* src) {
    auto hex = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    size_t di = 0;
    for (size_t i = 0; src[i] && di + 1 < dst_sz; ) {
        const char c = src[i];
        if (c == '%' && src[i + 1] && src[i + 2]) {
            const int hi = hex(src[i + 1]), lo = hex(src[i + 2]);
            if (hi >= 0 && lo >= 0) { dst[di++] = static_cast<char>((hi << 4) | lo); i += 3; continue; }
        }
        dst[di++] = (c == '+') ? ' ' : c;
        ++i;
    }
    dst[di] = '\0';
}

// Append a JSON-escaped string (SSIDs can contain quotes/backslashes/control bytes).
void JsonAppend(std::string& out, const char* s) {
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

// Escape a string for use inside a double-quoted HTML attribute or as element text.
void HtmlAttrAppend(std::string& out, const char* s) {
    for (const char* p = s; *p; ++p) {
        switch (*p) {
        case '&': out += "&amp;";  break;
        case '"': out += "&quot;"; break;
        case '<': out += "&lt;";   break;
        case '>': out += "&gt;";   break;
        default:  out += *p;
        }
    }
}

// Replace every occurrence of `from` with `to` (used to inject the scanned SSID <option>s).
void ReplaceAll(std::string& s, const char* from, const char* to) {
    const size_t flen = strlen(from), tlen = strlen(to);
    for (size_t pos = 0; (pos = s.find(from, pos)) != std::string::npos; pos += tlen)
        s.replace(pos, flen, to);
}

// Scan once and cache the result as both JSON (/scan) and <option> HTML (injected into GET /).
// Runs in ScanTask — never from an HTTP handler — so the scan's channel hop can't drop the
// SoftAP client that asked for it.
void DoScanAndCache() {
    wifi_scan_config_t sc = {};
    sc.show_hidden = false;
    sc.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    sc.scan_time.active.min = 100;
    sc.scan_time.active.max = 200;   // bound per-channel dwell -> shorter scan, less AP disruption

    esp_err_t r = ESP_FAIL;
    for (int attempt = 0; attempt < 5; ++attempt) {          // STA may not be ready right after start
        r = esp_wifi_scan_start(&sc, true /*block*/);
        if (r == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    if (r != ESP_OK) { ESP_LOGW(TAG, "scan failed: %s", esp_err_to_name(r)); return; }

    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 24) n = 24;
    std::string json = "[";
    std::string opts;
    int count = 0;
    if (n > 0) {
        wifi_ap_record_t* recs = static_cast<wifi_ap_record_t*>(calloc(n, sizeof(wifi_ap_record_t)));
        if (recs && esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
            for (uint16_t i = 0; i < n; ++i) {
                const char* ssid = reinterpret_cast<const char*>(recs[i].ssid);
                if (!ssid[0]) continue;                                      // skip hidden/empty
                bool dup = false;                                            // de-dup mesh/dual-band repeats
                for (uint16_t j = 0; j < i; ++j)
                    if (strcmp(ssid, reinterpret_cast<const char*>(recs[j].ssid)) == 0) { dup = true; break; }
                if (dup) continue;
                if (count) json += ',';
                json += "{\"ssid\":\"";
                JsonAppend(json, ssid);
                char b[48];
                snprintf(b, sizeof b, "\",\"rssi\":%d,\"lock\":%s}",
                         recs[i].rssi, recs[i].authmode == WIFI_AUTH_OPEN ? "false" : "true");
                json += b;
                opts += "<option value=\"";
                HtmlAttrAppend(opts, ssid);
                opts += "\">";
                HtmlAttrAppend(opts, ssid);
                opts += "</option>";
                ++count;
            }
        }
        free(recs);
    }
    json += "]";
    {
        std::lock_guard<std::mutex> lk(s_scan_mtx);
        s_scan_json = std::move(json);
        s_scan_options = std::move(opts);
    }
    ESP_LOGI(TAG, "provisioning scan: %d network(s) found", count);
}

void ScanTask(void*) {
    vTaskDelay(pdMS_TO_TICKS(600));   // let the STA interface settle after esp_wifi_start()
    DoScanAndCache();
    s_scan_task = nullptr;
    vTaskDelete(nullptr);
}

// Redirect any request to the portal root at the SoftAP IP (used for OS captive-portal probes).
esp_err_t SendRedirectToPortal(httpd_req_t* req) {
    char loc[32];
    snprintf(loc, sizeof loc, "http://%s/", s_ap_ip_str);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", loc);
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

// ── HTTP handlers ────────────────────────────────────────────────────────────────────────
esp_err_t RootGet(httpd_req_t* req) {
    // Serve portal.html with the cached scan injected at {{SSIDS}}; no live scan on the request path.
    std::string opts;
    { std::lock_guard<std::mutex> lk(s_scan_mtx); opts = s_scan_options; }
    std::string page(portal_html_start);   // EMBED_TXTFILES blob is NUL-terminated
    ReplaceAll(page, "{{SSIDS}}", opts.c_str());
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, page.c_str(), page.size());
}

// Browsers/OS keep polling /favicon.ico — answer 204 so they stop (instead of being redirected into
// loading the portal HTML as an icon, which just retries and spams the log).
esp_err_t FaviconGet(httpd_req_t* req) {
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

esp_err_t ScanGet(httpd_req_t* req) {
    std::string json;
    { std::lock_guard<std::mutex> lk(s_scan_mtx); json = s_scan_json; }   // cached; see DoScanAndCache
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t ProvisionPost(httpd_req_t* req) {
    char body[512];
    int received = 0;
    const int total = (req->content_len < static_cast<int>(sizeof body) - 1)
                          ? static_cast<int>(req->content_len) : static_cast<int>(sizeof body) - 1;
    while (received < total) {
        const int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
            return ESP_FAIL;
        }
        received += r;
    }
    body[received] = '\0';

    char enc[256], ssid[33] = {0}, pass[65] = {0};
    if (httpd_query_key_value(body, "ssid", enc, sizeof enc) == ESP_OK) UrlDecode(ssid, sizeof ssid, enc);
    if (httpd_query_key_value(body, "password", enc, sizeof enc) == ESP_OK) UrlDecode(pass, sizeof pass, enc);

    if (!ssid[0]) return SendRedirectToPortal(req);   // empty SSID -> back to the form

    {   // seed status so the first /status poll already shows "connecting" for this SSID
        std::lock_guard<std::mutex> lk(s_status_mtx);
        s_status.st = AL_PROV_CONNECTING;
        s_status.ip[0] = s_status.reason[0] = '\0';
        strncpy(s_status.ssid, ssid, sizeof(s_status.ssid) - 1);
        s_status.ssid[sizeof(s_status.ssid) - 1] = '\0';
    }
    ESP_LOGI(TAG, "portal: credentials submitted for '%s'", ssid);
    if (s_on_creds) s_on_creds(ssid, pass);           // transport kicks off the station connect

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, connecting_html_start, HTTPD_RESP_USE_STRLEN);
}

esp_err_t StatusGet(httpd_req_t* req) {
    char json[192];
    {
        std::lock_guard<std::mutex> lk(s_status_mtx);
        snprintf(json, sizeof json,
                 "{\"state\":\"%s\",\"ssid\":\"%s\",\"ip\":\"%s\",\"reason\":\"%s\"}",
                 StatusName(s_status.st), s_status.ssid, s_status.ip, s_status.reason);
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// Anything not matched above (OS probe URLs like /generate_204, /hotspot-detect.html, /ncsi.txt,
// and every other domain the phone resolves to us) -> redirect to the portal so it pops open.
esp_err_t Err404Redirect(httpd_req_t* req, httpd_err_code_t /*err*/) {
    return SendRedirectToPortal(req);
}

// ── Tiny DNS server: answer every A query with the SoftAP IP (captive-portal trigger) ────────
void DnsTask(void*) {
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "dns: socket() failed"); s_dns_task = nullptr; vTaskDelete(nullptr); return; }

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
        ESP_LOGE(TAG, "dns: bind(:53) failed");
        close(sock);
        s_dns_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    timeval tv = { .tv_sec = 1, .tv_usec = 0 };       // wake up periodically to check s_dns_run
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    uint8_t pkt[256];
    while (s_dns_run) {
        sockaddr_in from = {};
        socklen_t from_len = sizeof from;
        const int n = recvfrom(sock, pkt, sizeof pkt, 0, reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n < 12) continue;                          // timeout or a runt shorter than a DNS header
        if (pkt[2] & 0x80) continue;                   // already a response, ignore

        // Walk the (uncompressed) question name to find where qtype/qclass sit.
        int p = 12;
        while (p < n && pkt[p] != 0) {
            if (pkt[p] & 0xC0) { p = -1; break; }       // compressed name in a query: bail out
            p += pkt[p] + 1;
        }
        if (p < 0 || p + 5 > n) continue;               // malformed / truncated
        const int qtype = (pkt[p + 1] << 8) | pkt[p + 2];
        const int qend  = p + 5;                          // past the 0 byte + qtype(2) + qclass(2)

        // Build the response in place: set QR/AA, keep 1 question, add 1 answer for A queries.
        pkt[2] |= 0x80;                                   // QR = response
        pkt[2] |= 0x04;                                   // AA = authoritative
        pkt[3] = 0x00;                                    // RA=0, RCODE=0
        pkt[6] = 0x00; pkt[7] = (qtype == 1) ? 0x01 : 0x00;  // ANCOUNT
        pkt[8] = pkt[9] = pkt[10] = pkt[11] = 0x00;       // NSCOUNT / ARCOUNT

        int out = qend;
        if (qtype == 1 && qend + 16 <= static_cast<int>(sizeof pkt)) {
            const uint8_t answer[] = {
                0xC0, 0x0C,             // name -> pointer to the question at offset 12
                0x00, 0x01,             // TYPE  A
                0x00, 0x01,             // CLASS IN
                0x00, 0x00, 0x00, 0x00, // TTL 0 (do not let the phone cache it)
                0x00, 0x04,             // RDLENGTH 4
            };
            memcpy(pkt + out, answer, sizeof answer);
            out += sizeof answer;
            memcpy(pkt + out, &s_ap_ip.addr, 4);          // RDATA: SoftAP IP (already network order)
            out += 4;
        }
        sendto(sock, pkt, out, 0, reinterpret_cast<sockaddr*>(&from), from_len);
    }
    close(sock);
    s_dns_task = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

// ── Public API ───────────────────────────────────────────────────────────────────────────
extern "C" esp_err_t al_wifi_prov_start(const char* ap_ssid, al_prov_creds_cb_t on_creds) {
    if (s_httpd) return ESP_OK;                          // already running
    s_on_creds = on_creds;
    {
        std::lock_guard<std::mutex> lk(s_status_mtx);
        s_status = ProvStatus{};
    }
    { std::lock_guard<std::mutex> lk(s_scan_mtx); s_scan_json = "[]"; s_scan_options.clear(); }

    // Resolve the SoftAP gateway IP (for DNS answers + the redirect Location); fall back to the default.
    esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ip = {};
    if (ap && esp_netif_get_ip_info(ap, &ip) == ESP_OK && ip.ip.addr) {
        s_ap_ip = ip.ip;
    } else {
        esp_netif_str_to_ip4("192.168.4.1", &s_ap_ip);
    }
    esp_ip4addr_ntoa(&s_ap_ip, s_ap_ip_str, sizeof s_ap_ip_str);

    // DNS server (wildcard -> SoftAP IP).
    s_dns_run = true;
    if (xTaskCreate(DnsTask, "al_prov_dns", 3072, nullptr, 5, &s_dns_task) != pdPASS) {
        s_dns_run = false;
        ESP_LOGE(TAG, "failed to start DNS task");
        return ESP_ERR_NO_MEM;
    }

    // HTTP server.
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size      = 8192;                          // handlers render/serve pages + build JSON
    cfg.max_uri_handlers = 6;
    cfg.lru_purge_enable = true;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start HTTP server");
        s_dns_run = false;
        return ESP_FAIL;
    }
    const httpd_uri_t routes[] = {
        { .uri = "/",            .method = HTTP_GET,  .handler = RootGet,       .user_ctx = nullptr },
        { .uri = "/scan",        .method = HTTP_GET,  .handler = ScanGet,       .user_ctx = nullptr },
        { .uri = "/status",      .method = HTTP_GET,  .handler = StatusGet,     .user_ctx = nullptr },
        { .uri = "/favicon.ico", .method = HTTP_GET,  .handler = FaviconGet,    .user_ctx = nullptr },
        { .uri = "/provision",   .method = HTTP_POST, .handler = ProvisionPost, .user_ctx = nullptr },
    };
    for (const auto& r : routes) httpd_register_uri_handler(s_httpd, &r);
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, Err404Redirect);

    // The portal deliberately 404s every OS probe URL (we redirect them) and clients reset sockets
    // constantly — quiet those expected warnings so the serial log stays readable.
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);

    // One-shot scan now (before anyone connects) so the page can list nearby networks from cache.
    if (!s_scan_task) xTaskCreate(ScanTask, "al_prov_scan", 4096, nullptr, 4, &s_scan_task);

    ESP_LOGI(TAG, "captive portal up: join WiFi '%s' (open), then a page opens at http://%s",
             ap_ssid ? ap_ssid : "?", s_ap_ip_str);
    return ESP_OK;
}

extern "C" void al_wifi_prov_set_status(al_prov_status_t status, const char* ip, const char* reason) {
    std::lock_guard<std::mutex> lk(s_status_mtx);
    s_status.st = status;
    if (ip)     { strncpy(s_status.ip, ip, sizeof(s_status.ip) - 1);         s_status.ip[sizeof(s_status.ip) - 1] = '\0'; }
    if (reason) { strncpy(s_status.reason, reason, sizeof(s_status.reason) - 1); s_status.reason[sizeof(s_status.reason) - 1] = '\0'; }
}

extern "C" void al_wifi_prov_stop(void) {
    if (s_httpd) { httpd_stop(s_httpd); s_httpd = nullptr; }
    s_dns_run = false;                                   // the DNS task exits within its 1s recv timeout
    s_on_creds = nullptr;
}

extern "C" bool al_wifi_prov_active(void) { return s_httpd != nullptr; }
