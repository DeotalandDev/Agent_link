// ============================================================================
// agent_link — Connectivity Layer for Deotaland Agent Platform
// ============================================================================
// This module provides a reusable connectivity layer that allows devices
// to access the Deotaland Agent platform with minimal integration effort

// Design Philosophy:
//   - New hardware simply "declares capabilities + registers callbacks"
//   - Completely abstracts BLE/transport protocol details
//   - Agent → Device: via agent_output_cb_t (speech/display/vibration/actuators)
//   - Device → Agent: via agent_link_push_* (voice/buttons/sensors/battery)
//   - Only handles "transport + protocol + capability routing"
//   - Hardware actions are implemented in device callbacks → naturally cross-platform
//
// See agent_link_caps.h for capability flags and agent_link_io.h for generic I/O
// ============================================================================

#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "agent_link_caps.h"
#include "agent_link_io.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Protocol version identifier */
#define AGENT_LINK_PROTO_VERSION 1

/**
 * @brief Agent → Device output callbacks
 * 
 * These callbacks are invoked by the SDK when the Agent platform sends
 * commands or data to the device. All hardware operations should be
 * implemented within these callbacks.
 * 
 * @note All callbacks are optional — set to NULL if not applicable.
 * @note Callbacks may be invoked from SDK internal threads; avoid blocking
 */
typedef struct {
    /**
     * @brief Audio output: TTS stream from Agent
     * @param pcm16 PCM16 audio data (16kHz, mono)
     * @param bytes Size of data in bytes
     * @param ctx   User context pointer
     * @note Arrives in chunks; on_audio_end is called when stream finishes
     */
    void (*on_audio_out)(const uint8_t* pcm16, size_t bytes, void* ctx);

    /** @brief Called when audio stream ends */
    void (*on_audio_end)(void* ctx);

    /**
     * @brief Display text
     * @param utf8 UTF-8 encoded text string
     * @param ctx  User context
     */
    void (*on_show_text)(const char* utf8, void* ctx);
    /**
     * @brief Display image
     * @param rgb565_be Big-endian RGB565 image data
     * @param w        Image width in pixels
     * @param h        Image height in pixels
     * @param ctx      User context
     */
    void (*on_show_image)(const uint8_t* rgb565_be, uint16_t w, uint16_t h, void* ctx);

    /**
     * @brief Video downlink: encoded video frame from Agent
     * @param frame Encoded video frame (format negotiated by capabilities)
     * @param bytes Frame size in bytes
     * @param ctx   User context
     * @note WiFi-only (BLE bandwidth insufficient); arrives in chunks
     */
    void (*on_video_out)(const uint8_t* frame, size_t bytes, void* ctx);

    /** @brief Haptic feedback (vibration motor) */
    void (*on_haptic)(uint32_t duration_ms, void* ctx);

    /** @brief LED control (RGB color) */
    void (*on_led)(uint32_t rgb, void* ctx);

    /** @brief General-purpose actuator */
    void (*on_actuate)(uint16_t channel, int32_t value, void* ctx);

     /**
     * @brief Agent list from platform
     * @param json_utf8 JSON array of available Agents (UTF-8)
     * @param ctx       User context
     * @note Device should present a selection UI to the user
     */
    void (*on_agent_list)(const char* json_utf8, void* ctx);
    /**
     * @brief Command/query handler (for commands that require a response)
     * @param cmd     Command ID
     * @param payload Command payload
     * @param len     Payload length
     * @param resp    Response buffer to fill
     * @param resp_cap Response buffer capacity
     * @param resp_len Output: actual response length
     * @param ctx     User context
     * @return true if command recognized and handled; false otherwise
     * @note Return true → SDK responds with status=0 + your data
     *       Return false → SDK forwards to on_custom or returns empty ACK
     * @note 0x03 GetChargingStatus is auto-handled by SDK using cached battery data
     */
    bool (*on_command)(uint16_t cmd, const uint8_t* payload, size_t len,
                       uint8_t* resp, size_t resp_cap, size_t* resp_len, void* ctx);

     /**
     * @brief Custom/private command handler (fire-and-forget, no response)
     * @param cmd     Command ID
     * @param payload Command payload
     * @param len     Payload length
     * @param ctx     User context
     * @note SDK automatically replies with empty ACK
     */
    void (*on_custom)(uint16_t cmd, const uint8_t* payload, size_t len, void* ctx);

    /**
     * @brief Start/stop microphone capture on request from the Agent (App-initiated "listen").
     * @param start  true = begin capturing and stream ASR audio; false = stop.
     * @param max_ms Suggested max capture duration in ms (0 = until stopped); board-specific.
     * @param ctx    User context.
     * @note Optional; set only if AGENT_CAP_MIC is advertised. On start, the board drives its
     *       mic → agent_link_asr_start()/asr_push()/asr_end() loop. Triggered by command 0x3C/0x3D.
     */
    void (*on_listen)(bool start, uint32_t max_ms, void* ctx);

    void* ctx;  ///< Opaque pointer passed through to all callbacks
} agent_output_cb_t;

/**
 * @brief Connection state change callback
 * @param state New connection state
 * @param ctx   User context
 */
typedef void (*agent_state_cb_t)(agent_state_t state, void* ctx);

/**
 * @brief Transport backend selection
 * 
 * "One API, two transports" — same API works over BLE or WiFi.
 * Default is BLE (0) for backward compatibility.
 */
typedef enum {
    AGENT_TRANSPORT_BLE  = 0,  // BLE:GATT control + L2CAP voice; video not supported
    AGENT_TRANSPORT_WIFI = 1,  // WiFi:control + voice + video
    AGENT_TRANSPORT_BOTH = 2,  // Hybrid:BLE control/provisioning + WiFi media
} agent_transport_kind_t;

/**
 * @brief WiFi transport configuration
 * 
 * Used when transport includes WIFI. May be NULL and provisioned later via BLE.
 */
typedef struct agent_wifi_config_s {
    const char* ssid;      // WiFi SSID
    const char* password;  // WiFi password (can be NULL for open networks)
    const char* endpoint;  // Deotaland cloud endpoint (signaling/media), e.g., "wss://agent.deotaland.ai/..."
    const char* token;     // Device authentication token (can be NULL)
} agent_wifi_config_t;

/**
 * @brief Agent link configuration
 * 
 * Passed to agent_link_init() to configure the device's connection.
 */
typedef struct {
    const char*              device_name; ///< BLE advertising name / platform display name (required)
    uint32_t                 caps;         ///< agent_cap_t bitmask (required)
    const agent_output_cb_t* output;       ///< Agent→Device callbacks (NULL for pure sensor devices)
    agent_state_cb_t         on_state;     ///< Connection state callback (can be NULL)
    void*                    state_ctx;    ///< Context passed to on_state
    agent_transport_kind_t   transport;    ///< Transport backend (default 0 = BLE)
    const agent_wifi_config_t* wifi;        ///< WiFi config (when transport includes WIFI; can be NULL for later provisioning)
    
    // BLE Device Information Service (0x180A) strings
    const char*              manufacturer;  ///< Manufacturer name (0x2A29); NULL → "Deotaland"
    const char*              model;         ///< Model number (0x2A24); NULL → device_name
    const char*              firmware_rev;  ///< Firmware revision (0x2A26); NULL → "1.0.0"
} agent_link_config_t;

//=================================================================
// Lifecycle Management
//=================================================================

/** Initialize the agent link module */
esp_err_t     agent_link_init(const agent_link_config_t* cfg);

/** Start the connection (BLE advertising or WiFi connection) */
esp_err_t     agent_link_start(void);

/** Stop the connection and release resources */
void          agent_link_stop(void);

/** Get current connection state */
agent_state_t agent_link_state(void);

// ============================================================================
// Device → Agent: Input / Event / Status
// ============================================================================
// These functions are called by the device to report data to the Agent platform.
// They are safely ignored if the connection is not ready.
// ============================================================================


/**
 * @brief Push voice input stream
 * @param pcm16 PCM16 audio data (16kHz, 16-bit, mono)
 * @param bytes Data size in bytes
 * @note Call agent_link_voice_end() at the end of an utterance
 */
esp_err_t agent_link_push_voice(const uint8_t* pcm16, size_t bytes);

/** @brief Mark the end of a voice utterance */
esp_err_t agent_link_voice_end(void);

/**
 * @brief Start a real-time ASR audio stream (device → App).
 *
 * Opens the record-stream channel. The App transcribes the streamed audio live (ASR). 
 * agent_link is a transparent pipe: it transports exactly the bytes you push and 
 * owns only the transfer_id, the 0x52/0x53 framing, and L2CAP chunking + backpressure.
 * feed whatever audio format the App's ASR expects
 * 
 * @param name Stream label carried in the 0x52 event (UTF-8, no NUL). May be NULL.
 * @return ESP_OK once the stream is open; ESP_ERR_INVALID_STATE if the link / L2CAP channel is not ready.
 * @note Requires the App to have opened the L2CAP CoC (PSM 0x0081) after connecting. BLE transport only,
 *       one stream at a time. Push audio with agent_link_asr_push(), then close with agent_link_asr_end().
 */
esp_err_t agent_link_asr_start(const char* name);

/** @brief Push a chunk of ASR audio, streamed to the App over L2CAP (call agent_link_asr_start() first). */
esp_err_t agent_link_asr_push(const uint8_t* audio, size_t bytes);

/**
 * @brief End the ASR audio stream (BLE: event 0x53 StreamEnd with status + valid_bytes).
 * @param complete true = clean end (status=0); false = aborted/truncated (status=1).
 */
esp_err_t agent_link_asr_end(bool complete);

/** @brief Wire format of the bytes handed to agent_link_send_image(). */
typedef enum {
    AGENT_IMG_JPEG      = 0,  ///< JPEG-encoded (recommended over BLE; ~10-30KB per 240x240 frame)
    AGENT_IMG_RGB565_BE = 1,  ///< Raw big-endian RGB565, width*height*2 bytes (large; BLE-slow)
} agent_image_format_t;

/**
 * @brief Send a single still image (snapshot) to the App.
 *
 * A transparent, fire-and-forget one-shot: the SDK owns the transfer_id, the 0x54/0x55 framing,
 * and L2CAP chunking + backpressure; it does NOT encode — feed it whatever bytes the App expects
 * (encode to JPEG on the board, e.g. via esp_jpeg, before calling). The call returns as soon as the
 * image is queued; a background worker streams it and emits 0x55 StreamEnd when done.
 *
 * @param data  Image bytes (JPEG or raw RGB565 per @p fmt).
 * @param bytes Size of @p data in bytes.
 * @param fmt   Pixel/encoding format carried in the 0x54 event so the App can decode.
 * @param w     Image width in pixels (carried in 0x54; 0 if unknown).
 * @param h     Image height in pixels (carried in 0x54; 0 if unknown).
 * @return ESP_OK once queued; ESP_ERR_INVALID_STATE if the link / L2CAP image channel is not ready;
 *         ESP_ERR_INVALID_ARG on empty input.
 * @note BLE transport only.One image at a time, but independent of (concurrent with) the ASR audio stream.
 */
esp_err_t agent_link_send_image(const uint8_t* data, size_t bytes,
                                agent_image_format_t fmt, uint16_t w, uint16_t h);

/**
 * @brief Push a device→Agent event
 *
 * The event_id on the wire is the agent_event_t value. Use AGENT_EVT_CUSTOM (0x64) for a board-private packet
 * the board defines its own payload and the App matches on event_id 0x64
 * a dropped frame is not resent, so don't rely on it for state that must never desync
 * carry the explicit state in the payload rather than a bare toggle.
 * @param type Event type (from agent_event_t; AGENT_EVT_CUSTOM for board-private packets)
 * @param data Event payload (can be NULL if len == 0)
 * @param len  Payload length
 */
esp_err_t agent_link_push_event(agent_event_t type, const uint8_t* data, size_t len);

/**
 * @brief Report battery status
 * @param percent  Battery percentage (0-100)
 * @param charging true if charging
 * @note SDK internally deduplicates — only pushes when value changes /
 *       charging state changes / low-battery edge (<5%) is detected.
 *       If send fails, cache is not updated → next call retries.
 *       First call after connection forces a retransmission for initial sync.
 */
esp_err_t agent_link_report_battery(uint8_t percent, bool charging);

/**
 * @brief Report the user's selected Agent ID
 * @param agent_id The selected Agent ID
 */
esp_err_t agent_link_report_selected_agent(const char* agent_id);

/**
 * @brief Push an encoded video frame
 * @param frame    Encoded video frame (JPEG/H264, format negotiated)
 * @param bytes    Frame size in bytes
 * @param pts_ms   Presentation timestamp in milliseconds
 * @param keyframe true if this is an I-frame (keyframe)
 */
esp_err_t agent_link_push_video(const uint8_t* frame, size_t bytes, uint32_t pts_ms, bool keyframe);

/**
 * @brief End a video session (e.g., a call)
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t agent_link_video_end(void);

#ifdef __cplusplus
}
#endif
