#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>
#include "core/ring_buffer.hpp"
#include "core/types.hpp"
#include "dsp/noise_generator.hpp"
#include "channel/pair_buffer.hpp"

namespace openCREST {

// Per-receiver fan-in: sums the contributions from every PairBuffer that
// targets this receiver, adds receiver-specific ambient noise, converts to
// DAC samples, and writes to the receiver's rx_ring.
//
// Driven by the receiver-side ModemIO (which calls pull() before sending an
// RX packet to the modem). The receiver's clock is the modem DAC rate, so
// the read cadence is naturally driven by ModemIO::send_rx_data.
//
// Lifetime: incoming PairBuffer pointers must outlive the ReceiverMix.
class ReceiverMix {
public:
    ReceiverMix(std::vector<PairBuffer*> incoming,
                dsp::NoiseConfig         noise_cfg,
                CalibrationData          rcv_cal,
                uint32_t                 sample_rate);

    ReceiverMix(const ReceiverMix&)            = delete;
    ReceiverMix& operator=(const ReceiverMix&) = delete;

    // Pull `count` receiver-time samples:
    //   for each incoming PairBuffer p:
    //     p.read_advance(scratch, count)  → temp accumulator += scratch
    //   accumulator += noise_gen.generate(count)
    //   convert float → DAC bits, write to rx_ring (best effort).
    // Returns the number of DAC samples accepted by rx_ring (may be less
    // than `count` if rx_ring is full — caller may retry next tick).
    //
    // Always advances the PairBuffer read heads by `count` even if rx_ring
    // is too full to accept the full block, so the receiver clock keeps up
    // with wall time (the rx_ring drop is the visible loss).
    size_t pull(SPSCRingBuffer<uint16_t>& rx_ring, size_t count);

    // Diagnostics
    uint64_t rx_ring_drops() const { return rx_ring_drops_; }

private:
    std::vector<PairBuffer*> incoming_;
    dsp::NoiseGenerator      noise_gen_;
    CalibrationData          rcv_cal_;

    // Pre-allocated scratch. Sized for one PROCESSING_BLOCK_SIZE pull;
    // pull() asserts count <= scratch size.
    std::vector<float>    pair_scratch_;     // for each PairBuffer read
    std::vector<float>    sum_scratch_;      // running sum across pairs
    std::vector<float>    noise_scratch_;
    std::vector<uint16_t> dac_scratch_;

    uint64_t rx_ring_drops_ = 0;
};

} // namespace openCREST
