#pragma once
#include <cstdint>
#include <cstddef>
#include "protocol/packets.hpp"
#include "core/types.hpp"

namespace openCREST::protocol {

// Encode and decode USB packet buffers.
//
// Static methods handle individual packet encoding/decoding. Instance
// methods maintain per-stream sequence-number state. One ProtocolCodec
// instance per stream (TX or RX), owned by its I/O thread; no sharing.

class ProtocolCodec {
public:
    // Encode a data packet. `samples` must have at least `sample_count`
    // elements; if sample_count < DATA_SAMPLES_PER_PKT, remaining samples
    // are zero-padded. `buf_512` must be exactly 512 bytes.
    static void encode_data_packet(uint8_t* buf_512,
                                   uint16_t packet_id,
                                   const uint16_t* samples,
                                   size_t sample_count);

    // Decode a data packet into `samples` (capped at DATA_SAMPLES_PER_PKT).
    // `actual_samples` receives the count written. Always returns true (data
    // packets have no type discriminator beyond packet_id).
    static bool decode_data_packet(const uint8_t* buf_512,
                                   uint16_t& packet_id,
                                   uint16_t* samples,
                                   size_t max_samples,
                                   size_t& actual_samples);

    // Encode a command packet. `payload` may be nullptr when payload_len is
    // 0. Remaining bytes are zeroed.
    static void encode_command(uint8_t* buf_64,
                               CommandId cmd,
                               const uint8_t* payload = nullptr,
                               size_t payload_len = 0);

    // Returns false if buf_64[0] is not the STATUS type byte.
    static bool decode_status(const uint8_t* buf_64, StatusPayload& status);

    // Returns false if buf_64[0] is not the CALIBRATION type byte.
    static bool decode_calibration(const uint8_t* buf_64, CalibrationData& cal);

    // Used by MockTransport.
    static void encode_calibration(uint8_t* buf_64, const CalibrationData& cal);

    // Used by MockTransport.
    static void encode_status(uint8_t* buf_64, const StatusPayload& status);

    // Check received packet_id against expected. Returns true on match.
    // Either way, expected_id_ is advanced to received_id + 1 (wraps at
    // 65535 → 0) so one bad packet doesn't poison the stream.
    bool check_sequence(uint16_t received_id);

    void reset_sequence();

    uint64_t gap_count() const { return gap_count_; }

private:
    uint16_t expected_id_ = 0;
    uint64_t gap_count_   = 0;
};

} // namespace openCREST::protocol
