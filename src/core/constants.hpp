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

// Method-of-images geometric channel: highest reflection order (bounces) the
// image expansion supports and the resulting max path count. Order n adds two
// rays (except the direct path), so 1 + 2*kMaxImageOrder arrivals at full
// expansion. Order 2 (5 paths) is the default; higher orders are opt-in.
constexpr int    kMaxImageOrder          = 4;
constexpr size_t MAX_GEOMETRIC_PATHS     = 1 + 2 * kMaxImageOrder;  // 9

// Modem audio buffer (fixed by firmware).
constexpr uint16_t MODEM_AUDIO_BUFFER_CAPACITY = 16'384;

// Ring buffer sizing (in samples).
// TX ring: modem→host capture, ~1 s at 500 kSPS.
constexpr size_t TX_RING_CAPACITY        = 512 * 1024;
// RX ring: host→modem, only needs to cover the pacer's send slack.
// 64 K samples ≈ 130 ms at 500 kSPS (~128 KB).
constexpr size_t RX_RING_CAPACITY        = 64 * 1024;

} // namespace openCREST
