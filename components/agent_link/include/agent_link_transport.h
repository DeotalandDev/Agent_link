/**
 * @file agent_link_transport.h
 * @brief Transport layer abstraction for agent_link.
 * @details Defines the interface for transport backends (BLE, WiFi) and the
 *          stream types used for data-plane communication.
 *          Control-plane frames (commands/responses/events) are sent via
 *          send_ctrl(). Data-plane streams (voice, video, recording, file)
 *          are sent via stream_start/send_stream/stream_end.
 */


#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Data‑plane stream type
 * @note The transport backend selects the appropriate media channel:
 *       BLE uses different L2CAP channels, WiFi uses different WebRTC tracks
 */
typedef enum {
    AGENT_STREAM_VOICE = 0,    ///< Voice (microphone uplink / TTS downlink)
    AGENT_STREAM_VIDEO,        ///< Video (camera uplink / remote downlink) — WiFi only
    AGENT_STREAM_RECORDING,    ///< Recording file upload / real-time ASR audio (BLE L2CAP PSM 0x0081)
    AGENT_STREAM_FILE,         ///< File transfer / OTA
    AGENT_STREAM_IMAGE,        ///< Still-image snapshot (device → App); BLE L2CAP PSM 0x0082, events 0x54/0x55
} agent_stream_t;

/**
 * @brief Transport backend operation table
 * @note All functions must be implemented by the backend
 *       Control plane is reliable but low‑bandwidth (GATT, WS, MQTT)
 *       Data plane is high‑throughput (L2CAP, WebRTC)
 */
typedef struct agent_transport_s {
    /**
     * @brief Start the transport (advertise/connect)
     * @param impl Backend private context
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t (*start)(void* impl);

    /**
     * @brief Stop the transport (disconnect/release)
     * @param impl Backend private context
     */
    void      (*stop)(void* impl);

    /**
     * @brief Send a control frame (command/response/event)
     * @param impl  Backend private context
     * @param frame Frame buffer (header + payload)
     * @param len   Frame length
     * @return ESP_OK on success, error code otherwise
     * @note The control channel is reliable and ordered
     */
    esp_err_t (*send_ctrl)(void* impl, const uint8_t* frame, size_t len);

    /**
     * @brief Start a data‑plane stream session
     * @param impl     Backend private context
     * @param type     Stream type (voice/video/recording/file)
     * @param meta     Optional metadata (e.g., codec, sample rate)
     * @param meta_len Length of meta
     * @return ESP_OK on success, error code otherwise
     * @note The backend allocates a session ID and resets sequence numbers
     */
    esp_err_t (*stream_start)(void* impl, agent_stream_t type, const uint8_t* meta, size_t meta_len);

    /**
     * @brief Send a chunk of data over an active stream
     * @param impl Backend private context
     * @param type Stream type
     * @param data Data buffer
     * @param len  Data length
     * @return ESP_OK on success, error code otherwise
     * @note For BLE voice, this uses GATT Notify 0xFFA1 (event 0x40 VoiceChunk)
     *       Recording/files use L2CAP CoC; video is WiFi only
     */
    esp_err_t (*send_stream)(void* impl, agent_stream_t type, const uint8_t* data, size_t len);

    /**
     * @brief End a data‑plane stream session
     * @param impl     Backend private context
     * @param type     Stream type
     * @param complete true if the stream ended normally, false if aborted
     * @return ESP_OK on success, error code otherwise
     */
    /**
     * @param meta     Optional trailing metadata (60-byte final_header for the 0x53 event); NULL if none
     * @param meta_len Length of meta
     */
    esp_err_t (*stream_end)(void* impl, agent_stream_t type, bool complete, const uint8_t* meta, size_t meta_len);

    /**
     * @brief Check if the transport is ready (connected and encrypted)
     * @param impl Backend private context
     * @return true if ready, false otherwise
     */
    bool      (*is_ready)(void* impl);

    void*     impl;  // Backend private context
} agent_transport_t;

/**
 * @brief Get the BLE transport instance
 * @return Pointer to the BLE transport operation table
 * @note Control plane is fully implemented.
 *       Data plane (send_stream) is ready for voice via GATT Notify,
 *       recording/file via L2CAP (pending)
 */
agent_transport_t* agent_transport_ble(void);

/**
 * @brief Set BLE advertising name
 * @param name Device name (used in advertising and GAP)
 * @note Called by agent_link_init() with the device name from config
 */
void agent_transport_ble_set_name(const char* name);

/**
 * @brief Register callback for incoming control frames
 * @param cb Function called when a control frame is received (0xFFC1 writes)
 * @note Called by agent_link_init() to wire the core's OnCtrlFrame
 */
void agent_transport_ble_set_recv(void (*cb)(const uint8_t* data, size_t len));

/**
 * @brief Register callback for connection state changes
 * @param cb Function called when BLE connects or disconnects
 * @note Called by agent_link_init() to wire the core's OnConn
 */
void agent_transport_ble_set_conn(void (*cb)(bool connected));

/**
 * @brief Register callback for incoming data frames
 * @param cb Function called when a data frame is received
 * @note Called by agent_link_init() to wire the core's OnStreamRecv
 */
void agent_transport_ble_set_stream_recv(void (*cb)(agent_stream_t type, const uint8_t* data, size_t len));

/**
 * @brief Register callback for transport readiness
 * @param cb Function called when the transport becomes ready
 * @note Called by agent_link_init() to wire the core's OnReady
 */
void agent_transport_ble_set_ready(void (*cb)(void));

/**
 * @brief Get the current ATT MTU.
 * @return MTU size in bytes, or 0 if not connected.
 * @note Used by the core to adapt manifest fragment size.
 */
uint16_t agent_transport_ble_att_mtu(void);

/**
 * @brief Set device information for the standard Device Information Service (0x180A).
 * @param manufacturer Manufacturer name (0x2A29), NULL/empty keeps default.
 * @param model        Model number (0x2A24), NULL/empty keeps default.
 * @param firmware_rev Firmware revision (0x2A26), NULL/empty keeps default.
 * @note Called once during init.
 */
void agent_transport_ble_set_device_info(const char* manufacturer, const char* model, const char* firmware_rev);

/**
 * @brief Update the battery level in the standard Battery Service (0x180F, 0x2A19).
 * @param percent Battery percentage (0–100).
 * @note Stores the value for read and notifies if connected.
 *       Called by the core when battery changes.
 */
void agent_transport_ble_update_battery(uint8_t percent);

/**
 * @brief Get the WiFi transport instance.
 * @return Pointer to the WiFi transport operation table.
 */
agent_transport_t* agent_transport_wifi(void);

/** Forward declaration of WiFi config structure (defined in agent_link.h). */
struct agent_wifi_config_s;

/**
 * @brief Set WiFi connection parameters.
 * @param cfg Pointer to the WiFi configuration structure.
 * @note Called by agent_link_init() to pass the wifi config.
 */
void agent_transport_wifi_set_config(const struct agent_wifi_config_s* cfg);

/**
 * @brief Set the device name used as the SoftAP SSID prefix for WiFi provisioning.
 * @param name Device name; the captive-portal SoftAP is advertised as "<name>-XXXX".
 * @note Called by agent_link_init() with the device name from config.
 */
void agent_transport_wifi_set_name(const char* name);

/**
 * @brief Register callback for incoming control frames.
 * @param cb Function called when a control message is received (WS/DataChannel).
 * @note Called by agent_link_init() to wire the core's OnCtrlFrame.
 */
void agent_transport_wifi_set_recv(void (*cb)(const uint8_t* data, size_t len));

/**
 * @brief Register callback for connection state changes.
 * @param cb Function called when WiFi connects or disconnects.
 * @note Called by agent_link_init() to wire the core's OnConn.
 */
void agent_transport_wifi_set_conn(void (*cb)(bool connected));

/**
 * @brief Register callback for incoming data-plane data.
 * @param cb Function called when a data chunk arrives (e.g., WebRTC audio/video).
 * @note Called by agent_link_init() to wire the core's OnStreamData.
 */
void agent_transport_wifi_set_stream_recv(void (*cb)(agent_stream_t type, const uint8_t* data, size_t len));

#ifdef __cplusplus
}
#endif
