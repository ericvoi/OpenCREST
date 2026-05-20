#pragma once
#include <cstdint>
#include <cstddef>
#include "protocol/packets.hpp"
#include "core/types.hpp"

namespace openCREST::protocol {

// Encode and decode USB packet buffers.
//
// Static methods handle encoding/decoding of individual packets.
// Instance methods maintain per-stream sequence-number state.
//
// Thread-safety: one ProtocolCodec instance per stream (TX or RX),
// owned by the I/O thread. No sharing between threads.

class ProtocolCodec {
public:
    // -----------------------------------------------------------------------
    // Data packets (512-byte buffers)
    // -----------------------------------------------------------------------

    // Encode a data packet. `samples` must have at least `sample_count`
    // elements; if sample_count < DATA_SAMPLES_PER_PKT, remaining samples
    // in the packet are zero-padded. `buf_512` must be exactly 512 bytes.
    static void encode_data_packet(uint8_t* buf_512,
                                   uint16_t packet_id,
                                   const uint16_t* samples,
                                   size_t sample_count);

    // Decode a data packet. Fills `samples` with up to `max_samples` values
    // (capped at DATA_SAMPLES_PER_PKT). Sets `actual_samples` to the number
    // written. Always returns true (data packets have no type discriminator
    // beyond the packet_id).
    static bool decode_data_packet(const uint8_t* buf_512,
                                   uint16_t& packet_id,
                                   uint16_t* samples,
                                   size_t max_samples,
                                   size_t& actual_samples);

    // -----------------------------------------------------------------------
    // Control packets (64-byte buffers)
    // -----------------------------------------------------------------------

    // Encode a command packet. `payload` may be nullptr if payload_len == 0.
    // Remaining bytes in buf_64 are zeroed.
    static void encode_command(uint8_t* buf_64,
                               CommandId cmd,
                               const uint8_t* payload = nullptr,
                               size_t payload_len = 0);

    // Decode a status packet. Returns false if buf_64[0] != STATUS type byte.
    static bool decode_status(const uint8_t* buf_64, StatusPayload& status);

    // Decode a calibration response into CalibrationData. Returns false if
    // buf_64[0] != CALIBRATION type byte.
    static bool decode_calibration(const uint8_t* buf_64, CalibrationData& cal);

    // Encode a calibration response (used by MockTransport).
    static void encode_calibration(uint8_t* buf_64, const CalibrationData& cal);

    // Encode a status response (used by MockTransport).
    static void encode_status(uint8_t* buf_64, const StatusPayload& status);

    // -----------------------------------------------------------------------
    // Sequence-number tracking (per-stream instance state)
    // -----------------------------------------------------------------------

    // Check received packet_id against expected. Returns true if in sequence,
    // false if a gap or reorder was detected. Either way, `expected_id_` is
    // advanced to `received_id + 1` (wraps at 65535 → 0) so one bad packet
    // doesn't poison all subsequent checks.
    bool check_sequence(uint16_t received_id);

    void reset_sequence();

    uint64_t gap_count() const { return gap_count_; }

private:
    uint16_t expected_id_ = 0;
    uint64_t gap_count_   = 0;
};

} // namespace openCREST::protocol
