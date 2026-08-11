/**
 * @file agent_link_protocol.h
 * @brief Control‑plane protocol frame encoding/decoding.
 * @details 6‑byte header + payload:
 *          version(1)=0x01 | msg_type(1) | command_id(1) | sequence(1) | payload_len(2, little‑endian) | payload
 *          msg_type lower 7 bits: 0x01=Command, 0x02=Response, 0x03=Event; bit 0x80 = encrypted.
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace agentlink {

/** @name Protocol Constants */
constexpr uint8_t kVersion      = 0x01;     //Protocol version
constexpr uint8_t kMsgCommand   = 0x01;     //Message type: command
constexpr uint8_t kMsgResponse  = 0x02;     //Message type: response
constexpr uint8_t kMsgEvent     = 0x03;     //Message type: event
constexpr uint8_t kMsgEncrypted = 0x80;     //Encryption flag
constexpr size_t  kHeaderSize   = 6;        //Fixed header size in bytes

/**
 * @brief Decoded frame structure.
 * @note msg_type has the encryption bit stripped (0x01/0x02/0x03).
 */
struct Frame {
    uint8_t              msg_type;    // Message type (without encryption bit)
    uint8_t              command_id;  // Command / event ID
    uint8_t              sequence;    // Request‑response pairing sequence
    bool                 encrypted;   // True if the original frame had the encryption flag set
    std::vector<uint8_t> payload;     //Frame payload
};

/**
 * @brief Parse a raw byte stream into a Frame.
 * @param data Pointer to the input buffer.
 * @param len  Buffer length.
 * @param out  Output frame (filled on success).
 * @retval true  Frame is valid and was parsed successfully.
 * @retval false Frame is malformed (version mismatch, invalid length, or unknown type).
 */
bool ParseFrame(const uint8_t* data, size_t len, Frame& out);

/**
 * @brief Build a response frame (message_type=Response).
 * @param command_id Command ID.
 * @param sequence Sequence number.
 * @param status Status code.
 * @param error_code Error code.
 * @param extra Extra payload (optional).
 * @param extra_len Length of extra payload.
 * @return Vector containing the serialized frame.
 */
std::vector<uint8_t> BuildResponse(uint8_t command_id, uint8_t sequence,
                                   uint8_t status, uint16_t error_code,
                                   const uint8_t* extra = nullptr, size_t extra_len = 0);

/**
 * @brief Build an event frame (msg_type = Event, sequence = 0).
 * @param event_id Event ID.
 * @param payload  Event payload (can be nullptr if len == 0).
 * @param len      Payload length.
 * @return A byte vector containing the complete event frame.
 */
std::vector<uint8_t> BuildEvent(uint8_t event_id, const uint8_t* payload, size_t len);

}  // namespace agentlink
