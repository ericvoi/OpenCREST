#include "channel/receiver_mix.hpp"
#include "core/constants.hpp"
#include "core/sample_conversion.hpp"

#include <algorithm>
#include <cstring>

namespace openCREST {

ReceiverMix::ReceiverMix(std::vector<PairBuffer*> incoming,
                         dsp::NoiseConfig         noise_cfg,
                         CalibrationData          rcv_cal,
                         uint32_t                 sample_rate)
    : incoming_(std::move(incoming))
    , noise_gen_(noise_cfg, sample_rate)
    , rcv_cal_(rcv_cal)
    , pair_scratch_ (PROCESSING_BLOCK_SIZE, 0.0f)
    , sum_scratch_  (PROCESSING_BLOCK_SIZE, 0.0f)
    , noise_scratch_(PROCESSING_BLOCK_SIZE, 0.0f)
    , dac_scratch_  (PROCESSING_BLOCK_SIZE, 0)
{}

size_t ReceiverMix::pull(SPSCRingBuffer<uint16_t>& rx_ring, size_t count) {
    if (count == 0) return 0;

    // Chunk into scratch-sized blocks so very large `count` values work.
    size_t total_accepted = 0;
    while (count > 0) {
        const size_t chunk = std::min(count, sum_scratch_.size());

        // 1. Sum incoming PairBuffer contributions.
        std::memset(sum_scratch_.data(), 0, chunk * sizeof(float));
        for (auto* pb : incoming_) {
            // Missing samples (producer behind) stay at 0 from the memset,
            // so the receiver sees silence for the gap.
            const size_t got = pb->read_advance(pair_scratch_.data(), chunk);
            for (size_t i = 0; i < got; ++i) {
                sum_scratch_[i] += pair_scratch_[i];
            }
        }

        // 2. Add receiver-specific ambient noise.
        noise_gen_.generate(noise_scratch_.data(), chunk);
        for (size_t i = 0; i < chunk; ++i) {
            sum_scratch_[i] += noise_scratch_[i];
        }

        // 3. Convert to DAC samples (clamps; never wraps).
        const uint8_t dac_bits = rcv_cal_.dac_bits;
        for (size_t i = 0; i < chunk; ++i) {
            dac_scratch_[i] = float_to_dac(sum_scratch_[i], dac_bits);
        }

        // 4. Best-effort write to rx_ring; excess is counted as drops.
        const size_t accepted = rx_ring.write(dac_scratch_.data(), chunk);
        total_accepted += accepted;
        if (accepted < chunk) {
            rx_ring_drops_ += (chunk - accepted);
        }

        count -= chunk;
    }
    return total_accepted;
}

} // namespace openCREST
