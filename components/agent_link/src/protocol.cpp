// Control‑plane protocol frame encoding/decoding implementation
#include "protocol.h"
#include <cstring>

namespace agentlink {

bool ParseFrame(const uint8_t* data, size_t len, Frame& out) {
    if (!data || len < kHeaderSize) return false;

    const uint8_t version = data[0];
    if (version != kVersion) return false;

    const uint8_t raw_type = data[1];
    const uint8_t msg_type = raw_type & 0x7F;
    if (msg_type < kMsgCommand || msg_type > kMsgEvent) return false;

    const uint16_t payload_len = static_cast<uint16_t>(data[4] | (data[5] << 8));  // little‑endian
    if (kHeaderSize + payload_len != len) return false;  // length must be self‑consistent

    out.msg_type   = msg_type;
    out.command_id = data[2];
    out.sequence   = data[3];
    out.encrypted  = (raw_type & kMsgEncrypted) != 0;
    out.payload.assign(data + kHeaderSize, data + kHeaderSize + payload_len);
    //Note: encryption is not implemented; the encrypted flag is retained
    return true;
}

static void WriteHeader(std::vector<uint8_t>& f, uint8_t msg_type, uint8_t id,
                        uint8_t seq, uint16_t payload_len) {
    f.push_back(kVersion);
    f.push_back(msg_type);
    f.push_back(id);
    f.push_back(seq);
    f.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    f.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
}

std::vector<uint8_t> BuildResponse(uint8_t command_id, uint8_t sequence,
                                   uint8_t status, uint16_t error_code,
                                   const uint8_t* extra, size_t extra_len) {
    const size_t ack = 4;  // acked_cmd + status + err(2, LE)
    const uint16_t payload_len = static_cast<uint16_t>(ack + extra_len);
    std::vector<uint8_t> f;
    f.reserve(kHeaderSize + payload_len);
    WriteHeader(f, kMsgResponse, command_id, sequence, payload_len);
    f.push_back(command_id);
    f.push_back(status);
    f.push_back(static_cast<uint8_t>(error_code & 0xFF));
    f.push_back(static_cast<uint8_t>((error_code >> 8) & 0xFF));
    if (extra && extra_len > 0) f.insert(f.end(), extra, extra + extra_len);
    return f;
}

std::vector<uint8_t> BuildEvent(uint8_t event_id, const uint8_t* payload, size_t len) {
    std::vector<uint8_t> f;
    f.reserve(kHeaderSize + len);
    WriteHeader(f, kMsgEvent, event_id, /*sequence=*/0, static_cast<uint16_t>(len));
    if (payload && len > 0) f.insert(f.end(), payload, payload + len);
    return f;
}

}  // namespace agentlink
