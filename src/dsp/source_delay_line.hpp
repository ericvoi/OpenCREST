#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace openCREST::dsp {

// Source-sample reservoir for the per-tap Farrow used by every tap-based
// channel model (geometric, replay, ...).
//
// The producer (TapSourceRenderer) writes source-rate samples 1:1 as they
// arrive; each per-tap consumer reads at a fractional position evolving
// block-to-block according to that tap's time-varying delay. This is the
// core piece of read-style state in the tap-source rendering pipeline.
//
// Coordinates: positions are absolute uint64 source-sample counters.
// The internal ring is power-of-two with mask_ = capacity − 1, so an
// absolute position maps to a physical slot via `pos & mask`.
//
// Lifetime: between clear() calls the producer cursor advances
// monotonically and may exceed `capacity` — wrap is handled by masking.
// Each TapSource clamps its tap delays to the envelope the renderer sized
// this line for, so read positions never lag the producer by more than
// `capacity − 1` source samples.
//
// read_at(pos) does 4-tap Catmull-Rom cubic interpolation. The caller
// must ensure
//   1.0 ≤ pos ≤ producer_position() − 2
// and that the ring still holds the four samples spanning
// floor(pos)-1 .. floor(pos)+2 (true while pos ≥
// producer_position() − capacity + 1).
class SourceDelayLine {
public:
    SourceDelayLine() = default;

    // Allocate (or reallocate) the ring. capacity_samples is rounded up
    // to the next power of two if not already one. Init-time only.
    void resize(size_t capacity_samples);

    // Zero the ring and reset the producer cursor. Called on every
    // message boundary so each message starts with a clean buffer.
    void clear();

    // Append `n` samples at the producer cursor and advance it by n.
    void write(const float* in, size_t n);

    // Absolute source-sample count written since the last clear().
    uint64_t producer_position() const { return producer_position_; }

    // Physical ring capacity in samples (power of two).
    size_t capacity() const { return buffer_.size(); }

    // Cubic interpolation at fractional source-sample position pos.
    // See class comment for the safe range.
    float read_at(double pos) const;

private:
    std::vector<float> buffer_;
    size_t   mask_              = 0;
    uint64_t producer_position_ = 0;
};

} // namespace openCREST::dsp
