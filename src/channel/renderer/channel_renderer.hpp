#pragma once

#include <cstddef>

#include "channel/pair_buffer.hpp"

namespace openCREST {

// Rendering strategy for one directed channel: turns source-rate samples
// into receiver-time contributions scattered into a PairBuffer.
//
// A renderer is chosen once at construction by the channel model (see
// channel/model/channel_model.hpp) and never changes. Two implementations
// exist today:
//
//   StaticRenderer     — bulk FarrowResampler for Doppler, then integer-offset
//                        scatter of a fixed complex tap list (Hilbert pair
//                        when any tap has non-zero phase).
//   TapSourceRenderer  — SourceDelayLine + per-tap Farrow reads, with the tap
//                        set re-queried from a TapSource at each block's start
//                        and end times. Doppler emerges from time-varying tap
//                        delays rather than a bulk resampling ratio.
//
// The two differ in more than their tap supply — they are genuinely different
// signal paths — which is why the seam is here rather than at TapSource alone.
//
// Sizing contract: base_delay_samples() and max_tap_delta_samples() are fixed
// at construction and are what ChannelEngine sizes PairBuffers from. The
// renderer must never write beyond
//   write_origin + source_samples_processed + max_tap_delta_samples().
//
// Threading: one renderer is owned by one Channel, driven by exactly one
// SourceWorker thread. process() must be allocation-free.
class ChannelRenderer {
public:
    virtual ~ChannelRenderer() = default;

    // Source modem entered TX: reset per-message state. The PairBuffer's own
    // message bookkeeping is handled by Channel, not here.
    virtual void on_message_start() = 0;

    // Process up to `count` source samples ([-1,+1] float). Returns the number
    // of samples consumed; the caller retains the unconsumed tail.
    virtual size_t process(const float* samples, size_t count,
                           PairBuffer& pair_buffer) = 0;

    // Source modem left TX: drain filter histories and expose the multipath
    // tail so trailing arrivals reach the receiver.
    virtual void on_message_end(PairBuffer& pair_buffer) = 0;

    // Conservative upper bound on input samples to batch per process() call.
    virtual size_t input_needed_for_batch() const = 0;

    // Base propagation delay in source-rate samples.
    virtual size_t base_delay_samples() const = 0;

    // Maximum tap delay in samples relative to the base delay.
    virtual size_t max_tap_delta_samples() const = 0;

    // Instantaneous Doppler ratio at the current intra-message time.
    // Diagnostics only.
    virtual double current_doppler_ratio() const = 0;
};

} // namespace openCREST
