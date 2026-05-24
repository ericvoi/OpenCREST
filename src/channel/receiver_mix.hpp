#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>
#include "core/ring_buffer.hpp"
#include "core/types.hpp"
#include "dsp/noise_generator.hpp"
#include "channel/pair_buffer.hpp"

namespace openCREST {

// Per-receiver fan-in: sums every PairBuffer that targets this receiver,
// adds ambient noise, converts to DAC samples, writes to rx_ring.
//
// Driven by the receiver's ModemIO::send_rx_data; the read cadence is
// naturally the DAC rate.
//
// Incoming PairBuffer pointers must outlive the ReceiverMix.
class ReceiverMix {
public:
    ReceiverMix(std::vector<PairBuffer*> incoming,
                dsp::NoiseConfig         noise_cfg,
                CalibrationData          rcv_cal,
                uint32_t                 sample_rate);

    ReceiverMix(const ReceiverMix&)            = delete;
    ReceiverMix& operator=(const ReceiverMix&) = delete;

    // Pull `count` receiver-time samples: sum each incoming PairBuffer,
    // add noise, convert to DAC bits, best-effort write to rx_ring.
    // Returns the number of DAC samples accepted by rx_ring (may be less
    // than `count`).
    //
    // Always advances PairBuffer read heads by `count` even if rx_ring
    // is full, so the receiver clock keeps up with wall time. The rx_ring
    // drop is the visible loss.
    size_t pull(SPSCRingBuffer<uint16_t>& rx_ring, size_t count);

    uint64_t rx_ring_drops() const { return rx_ring_drops_; }

private:
    std::vector<PairBuffer*> incoming_;
    dsp::NoiseGenerator      noise_gen_;
    CalibrationData          rcv_cal_;

    // Pre-allocated scratch, one PROCESSING_BLOCK_SIZE block.
    std::vector<float>    pair_scratch_;
    std::vector<float>    sum_scratch_;
    std::vector<float>    noise_scratch_;
    std::vector<uint16_t> dac_scratch_;

    uint64_t rx_ring_drops_ = 0;
};

} // namespace openCREST
