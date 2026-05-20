#pragma once
#include <cstddef>
#include <cstdint>

namespace openCREST {

constexpr size_t MAX_MODEMS              = 6;
constexpr size_t MAX_TAPS_PER_CHANNEL    = 32;
constexpr size_t DEFAULT_TAPS            = 10;
constexpr size_t DATA_PACKET_SIZE        = 512;    // bytes
constexpr size_t CONTROL_PACKET_SIZE     = 64;     // bytes
constexpr size_t PACKET_HEADER_SIZE      = 2;      // packet_id (uint16_t LE)
constexpr float  MAX_MULTIPATH_DELAY_S   = 0.200f; // 200 ms
constexpr float  TR_SETTLING_TIME_S      = 0.200f; // 200 ms
constexpr size_t PROCESSING_BLOCK_SIZE   = 256;    // samples per processing tick

// Modem audio buffer (always 16384 per firmware spec §4)
constexpr uint16_t MODEM_AUDIO_BUFFER_CAPACITY = 16'384;

// Ring buffer sizing (in samples)
// TX ring: buffers modem→host capture before channel engine consumes it (~1 s)
constexpr size_t TX_RING_CAPACITY        = 512 * 1024;
// RX ring: in Phase 2, the propagation history lives in PairBuffers; rx_ring
// only needs to cover the BufferPacer's send slack (~tens of ms is plenty).
// 64 K samples ≈ 130 ms at 500 kSPS (~128 KB).
constexpr size_t RX_RING_CAPACITY        = 64 * 1024;

} // namespace openCREST
