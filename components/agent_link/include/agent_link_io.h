#pragma once
/**
 * @file agent_link_io.h
 * @brief Generic device I/O: arbitrary sensor reporting and arbitrary actuator control.
 * @details Sensors and actuators are an open set (gyroscope, temperature/humidity,
 *          encoder, joystick, light, smoke, PIR, thermistor, current, ...; buzzer,
 *          servo, relay, LED, ...). To avoid "one API per kind", the model is
 *          self-describing endpoints over a generic channel:
 *            1. At startup, declare each endpoint with agent_link_register_io()
 *               (id / direction / type / unit / description / params). The SDK
 *               collects them into a manifest sent to the Agent on connect, from
 *               which the cloud derives MCP tools/resources.
 *            2. Sensor readings go through agent_link_push_reading(id, value):
 *               one reporting channel, distinguished by id.
 *            3. Actuator commands from the Agent are routed by id to the
 *               registered agent_io_actuate_cb_t callback.
 *          Semantics ("temperature", "buzzer") live only in the description
 *          strings; the SDK moves bytes and routes by id, so adding a new sensor
 *          needs no SDK changes. See docs/device-io.md. Rich media (screen,
 *          speaker, camera) still use the dedicated callbacks in agent_link.h.
 */
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief I/O endpoint direction. */
typedef enum {
    AGENT_IO_IN  = 0,   ///< Sensor: device → Agent (reports readings)
    AGENT_IO_OUT = 1,   ///< Actuator: Agent → device (receives commands)
} agent_io_dir_t;

/** @brief Data type of a reading or parameter (how the value buffer is interpreted). */
typedef enum {
    AGENT_VAL_BOOL = 0, ///< Boolean, 1 byte (switch / trigger, e.g. PIR)
    AGENT_VAL_I32,      ///< Signed 32-bit (encoder count / servo angle, ...)
    AGENT_VAL_U16,      ///< Unsigned 16-bit (raw ADC, e.g. light sensor)
    AGENT_VAL_F32,      ///< Single-precision float (physical quantity: temperature / current, ...)
    AGENT_VAL_VEC2,     ///< 2D vector, float[2] (joystick x, y)
    AGENT_VAL_VEC3,     ///< 3D vector, float[3] (IMU / gyroscope / accelerometer)
    AGENT_VAL_RGB,      ///< Color 0x00RRGGBB (packed into a u32)
    AGENT_VAL_BLOB,     ///< Raw bytes (custom / composite actuator parameters)
    AGENT_VAL_STR,      ///< UTF-8 string, variable length (screen text / string readings)
} agent_val_t;

/** @brief Endpoint visibility to the AI vs the user/companion app. */
typedef enum {
    AGENT_AUD_AI   = 0, ///< LLM-callable (default): exposed as a normal MCP tool/resource
    AGENT_AUD_USER = 1, ///< User/app only: hidden from the LLM (e.g. reboot / factory reset / OTA)
} agent_audience_t;

/** @brief How an input endpoint reports readings (a hint for the Agent). */
typedef enum {
    AGENT_EVT_PERIODIC  = 0, ///< Periodic at rate_hz (default)
    AGENT_EVT_ON_CHANGE = 1, ///< Only when the value changes
    AGENT_EVT_THRESHOLD = 2, ///< Only when a threshold / alarm is crossed
} agent_evt_mode_t;

/**
 * @brief Self-description of one I/O endpoint.
 * @note Provided by the device at register time and kept long-term; use static storage.
 */
typedef struct {
    const char*    id;          ///< Unique id, e.g. "temp0" (required)
    agent_io_dir_t dir;         ///< IN = sensor / OUT = actuator (required)
    const char*    kind;        ///< Semantic type, e.g. "temperature" / "imu.gyro" / "buzzer" (required)
    agent_val_t    value;       ///< Data type (required)
    const char*    unit;        ///< Unit, e.g. "C" / "%RH" / "dps" / "A" (may be NULL)
    const char*    desc;        ///< Human-readable description for the LLM, e.g. "ambient temperature" (may be NULL)
    float          range_min;   ///< Range lower bound (range_min == range_max means unspecified)
    float          range_max;   ///< Range upper bound
    uint16_t       rate_hz;     ///< Suggested reporting rate in Hz (0 = event-driven / irregular)
    const char*    args_schema; ///< Actuator parameter schema (JSON string, OUT endpoints only; may be NULL)
    // ── Extended self-description (all optional; zero-initialized defaults are sensible) ──
    const char*      display_name; ///< Human-friendly name for UIs (may be NULL)
    agent_audience_t audience;     ///< AI-callable (default 0) or user-only (hidden from the LLM)
    const char*      enum_json;    ///< Allowed discrete values, JSON array string e.g. ["off","low","high"] (may be NULL)
    const char*      default_json; ///< Default value as a JSON literal string (may be NULL)
    agent_evt_mode_t event;        ///< Reporting semantics (default 0 = periodic)
} agent_link_io_desc_t;

/**
 * @brief Actuator callback: invoked by id when the Agent sends a command for an endpoint.
 * @param id   Endpoint id the command targets.
 * @param args Endpoint-specific arguments (interpreted per args_schema / value).
 * @param len  Argument length in bytes.
 * @param ctx  User context passed at registration.
 * @note Implementations should return quickly.
 */
typedef void (*agent_io_actuate_cb_t)(const char* id, const uint8_t* args, size_t len, void* ctx);

/**
 * @brief Register an I/O endpoint.
 * @param desc Endpoint descriptor (must stay valid; use static storage).
 * @param cb   Actuator command handler; pass NULL for sensors.
 * @param ctx  User context passed back to @p cb.
 * @return ESP_OK on success, error code otherwise.
 * @note Call before agent_link_start(). The SDK serializes all registered
 *       endpoints into a manifest sent to the Agent on connect.
 */
esp_err_t agent_link_register_io(const agent_link_io_desc_t* desc,
                                 agent_io_actuate_cb_t cb, void* ctx);

/**
 * @brief Report one sensor reading (by id + typed value).
 * @param id    Endpoint id.
 * @param value Pointer to data matching this endpoint's agent_val_t.
 * @param len   Value length in bytes.
 * @return ESP_OK on success, error code otherwise.
 * @note Small readings go over the control plane; aggregate high-rate batches
 *       before calling (a data-plane path may be added later).
 */
esp_err_t agent_link_push_reading(const char* id, const void* value, size_t len);

/**
 * @brief Notify the Agent that the I/O manifest changed (endpoints added/removed at runtime).
 * @return ESP_OK on success, error code otherwise.
 * @details Bumps the manifest revision, emits a 0x1A ManifestChanged event carrying the new
 *          revision, and re-sends the full manifest (0x18). Call after registering additional
 *          endpoints once the link may already be up. Safe to call when disconnected.
 */
esp_err_t agent_link_notify_manifest_changed(void);

#ifdef __cplusplus
}
#endif
