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

    // Process in chunks no larger than the scratch buffers. This caps
    // per-call memory use and lets very large `count` values work even
    // though scratch was sized for one PROCESSING_BLOCK_SIZE block.
    size_t total_accepted = 0;
    while (count > 0) {
        const size_t chunk = std::min(count, sum_scratch_.size());

        // 1. Sum incoming PairBuffer contributions (zero scratch first).
        std::memset(sum_scratch_.data(), 0, chunk * sizeof(float));
        for (auto* pb : incoming_) {
            // PairBuffer::read_advance returns however many samples were
            // available — the producer's clock may be slightly behind, in
            // which case the missing samples are silence (0). Guarantee
            // exactly `chunk` summed samples by zero-filling the gap.
            const size_t got = pb->read_advance(pair_scratch_.data(), chunk);
            for (size_t i = 0; i < got; ++i) {
                sum_scratch_[i] += pair_scratch_[i];
            }
            // If got < chunk, those positions stay at 0 from the memset.
        }

        // 2. Add receiver-specific ambient noise.
        noise_gen_.generate(noise_scratch_.data(), chunk);
        for (size_t i = 0; i < chunk; ++i) {
            sum_scratch_[i] += noise_scratch_[i];
        }

        // 3. Convert float → DAC samples (clamps; never wraps).
        const uint8_t dac_bits = rcv_cal_.dac_bits;
        for (size_t i = 0; i < chunk; ++i) {
            dac_scratch_[i] = float_to_dac(sum_scratch_[i], dac_bits);
        }

        // 4. Best-effort write to rx_ring. If the ring is full, the reader
        //    is too slow; we drop the excess but still advanced the
        //    PairBuffer read heads above.
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
