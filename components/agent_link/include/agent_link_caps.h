// agent_link Capability & Event Vocabulary

// This header defines the shared "language" between the device and the
// Deotaland Agent platform. It includes:
//   - Device capability flags (what hardware the device has)
//   - Input event types (what the device can report to Agent)
//   - Connection states (for monitoring link status)

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Device capability bits
 * 
 * These flags define the hardware capabilities of the device
 * Multiple capabilities can be combined using bitwise OR
 * 
 * @example
 * uint32_t caps = AGENT_CAP_MIC | AGENT_CAP_SPEAKER | AGENT_CAP_SCREEN
 */
typedef enum {
    AGENT_CAP_MIC       = 1u << 0,  // Audio input (microphone stream → Agent)
    AGENT_CAP_SPEAKER   = 1u << 1,  // Audio output (Agent sends TTS / beeps)
    AGENT_CAP_SCREEN    = 1u << 2,  // Display (show text / images)
    AGENT_CAP_BUTTON    = 1u << 3,  // Button input
    AGENT_CAP_HAPTIC    = 1u << 4,  // Motor / vibration feedback
    AGENT_CAP_BATTERY   = 1u << 5,  // Battery level / charging status reporting
    AGENT_CAP_LED       = 1u << 6,  // Controllable LED
    AGENT_CAP_SENSOR    = 1u << 7,  // General-purpose sensors (temp/humidity/light/PIR/gas…)
    AGENT_CAP_ACTUATOR  = 1u << 8,  // General-purpose actuators (GPIO/servo/relay…)
    AGENT_CAP_RECORDING = 1u << 9,  // Recording to storage + upload (meetings/memos)
    AGENT_CAP_CAMERA    = 1u << 10, // Camera (BLE: still-image snapshot upload via L2CAP; WiFi: video)
} agent_cap_t;

/**
 * @brief Device-to-Agent input event types
 * 
 * These values are used as the @p type parameter in agent_link_push_event()
 * to indicate the type of event being reported from the device to the Agent
 */
typedef enum {
    AGENT_EVT_BUTTON   = 1,    ///< Button: data suggested as {uint8 button_id, uint8 action}
    AGENT_EVT_SENSOR   = 2,    ///< Sensor reading: data is device-specific (JSON or TLV recommended)
    AGENT_EVT_WAKEWORD = 3,    ///< Wake word detected: data may include the matched word
    AGENT_EVT_CUSTOM   = 100,  ///< Device-private events (user-defined, 0x64+)
} agent_event_t;

/**
 * @brief Link status states reported by agent_link_state()
 * 
 * These values are returned by agent_link_state() to indicate the
 * current connection status between the device and the Agent platform
 */
typedef enum {
    AGENT_STATE_DISCONNECTED = 0,  //Not connected to Agent platform
    AGENT_STATE_CONNECTED,   // Link established (unencrypted)
    AGENT_STATE_READY,       // Encrypted + capability handshake complete, ready for data
} agent_state_t;

#ifdef __cplusplus
}
#endif
