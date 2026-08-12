/**
 * @file wifi_provision.h
 * @brief SoftAP captive-portal WiFi provisioning for the WiFi transport
 * @details When the device has no stored WiFi credentials it broadcasts an open SoftAP
 *          The user connects to it; a DNS wildcard + HTTP redirect make the phone auto-open a configuration page
 *          The user picks their home WiFi and enters the password,the transport then joins as a station
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Connection progress shown by the portal's /status endpoint. */
typedef enum {
    AL_PROV_IDLE = 0,     ///< Waiting for the user to submit credentials.
    AL_PROV_CONNECTING,   ///< Credentials received; the station is attempting to join.
    AL_PROV_CONNECTED,    ///< Station obtained an IP — provisioning succeeded.
    AL_PROV_FAILED,       ///< Station failed to join with the submitted credentials.
} al_prov_status_t;

/**
 * @brief User submitted WiFi credentials from the portal page.
 * @param ssid     Selected network SSID (NUL-terminated).
 * @param password Password (NUL-terminated; empty for an open network).
 * @note Both strings are only valid for the duration of the call — copy them out.
 *       Invoked from the HTTP server task; return quickly (kick off the connect asynchronously).
 */
typedef void (*al_prov_creds_cb_t)(const char* ssid, const char* password);

/**
 * @brief Start the captive portal (DNS redirect + HTTP config server).
 * @param ap_ssid   The SoftAP SSID the user sees (shown on the page for reassurance).
 * @param on_creds  Called when the user submits SSID/password.
 * @return ESP_OK on success, error code otherwise.
 * @note The caller (transport) must already have the SoftAP up in AP or APSTA mode.
 */
esp_err_t al_wifi_prov_start(const char* ap_ssid, al_prov_creds_cb_t on_creds);

/**
 * @brief Update the status the portal page polls after the user submits.
 * @param status New status.
 * @param ip     IP string when connected (may be NULL).
 * @param reason Short human-readable failure reason when failed (may be NULL).
 */
void al_wifi_prov_set_status(al_prov_status_t status, const char* ip, const char* reason);

/** @brief Tear down the portal (stops HTTP + DNS). Does not change WiFi mode. */
void al_wifi_prov_stop(void);

/** @brief Whether the portal is currently running. */
bool al_wifi_prov_active(void);

#ifdef __cplusplus
}
#endif
