#pragma once

#include <cstddef>

namespace openCREST {

// One tap emitted by a TapSource at a given intra-message time.
struct SourcedTap {
    // Excess delay over the channel's base propagation delay, in fractional
    // source-rate samples. Always >= 0.
    double delta_samples_frac = 0.0;
    // Real linear amplitude, model-relative: acoustic path gain only. The
    // Channel multiplies its AFE-electrical chain gain on top. Tap phase is
    // encoded in the fractional delay, so amplitude carries no sign.
    float  gain               = 0.0f;
};

// Per-block tap provider for the read-style rendering pipeline
// (SourceDelayLine + PerTapFarrow). Implementations:
//   GeometricTapSource — analytic method-of-images scene at R(t)
//   ReplayTapSource    — measured trajectory file (.octt)
// A future live source (e.g. AUV digital twin) fits by answering taps_at()
// from a fixed-lag buffer of externally supplied CIR ground-truth points.
//
// Contract:
//   - taps_at() is called twice per <=256-sample block (block start/end
//     times); it must be allocation-free and lock-free.
//   - The tap count is stable within one message; delay and amplitude
//     evolve continuously in t (the renderer lerps across the block, so a
//     step between adjacent calls produces an audible discontinuity).
//   - t is intra-message time in seconds since message start, at the
//     receiver's nominal sample clock.
class TapSource {
public:
    virtual ~TapSource() = default;

    // Channel::on_message_start forwards here: pin or advance the source's
    // time origin for the coming message.
    virtual void on_message_start() = 0;

    // Fill out[0..capacity) with the tap set at intra-message time
    // t_seconds. Returns the number of taps written
    // (min(active taps, capacity)).
    virtual size_t taps_at(double t_seconds, SourcedTap* out,
                           size_t capacity) = 0;

    // Init-time sizing; constant over the source lifetime.
    virtual size_t tap_count_max()         const = 0;  // <= MAX_TAPS_PER_CHANNEL
    virtual size_t max_tap_delta_samples() const = 0;  // PairBuffer extent
};

} // namespace openCREST
