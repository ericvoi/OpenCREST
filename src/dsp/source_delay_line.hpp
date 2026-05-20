#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace openCREST::dsp {

// Source-sample reservoir for the geometric-mode per-tap Farrow.
//
// The producer (Channel) writes source-rate samples 1:1 as they arrive;
// each per-tap "consumer" reads at a fractional position evolving from
// block to block according to that tap's time-varying delay. The
// SourceDelayLine is the only piece of read-style state in the
// geometric pipeline.
//
// Coordinate system
// -----------------
// Positions are absolute uint64-style source-sample counters. The
// internal ring is power-of-two and `mask_` = capacity − 1, so an
// absolute position maps to a physical slot via `pos & mask`.
//
// Lifetime invariant
// ------------------
// Between calls to clear() the producer cursor advances monotonically
// and may exceed `capacity` — wrap is handled by masking. A
// `recompute_geometric_taps` clamp in Channel keeps reads inside the
// configured envelope, so read positions never lag the producer by
// more than `capacity − 1` source samples.
//
// Interpolation
// -------------
// `read_at(pos)` performs 4-tap Catmull-Rom (Hermite) cubic
// interpolation matching FarrowResampler::interpolate (same Horner
// form). For Catmull-Rom safety the caller must ensure
//   1.0 ≤ pos ≤ producer_position() − 2
// and that the ring still holds the four samples spanning
// floor(pos)-1 .. floor(pos)+2 (true while pos ≥
// producer_position() − capacity + 1).
class SourceDelayLine {
public:
    SourceDelayLine() = default;

    // Allocate (or reallocate) the underlying ring. capacity_samples is
    // rounded up to the next power of two if not already one. Init-time
    // only; never call from the hot path.
    void resize(size_t capacity_samples);

    // Zero the ring and reset the producer cursor. Called by Channel on
    // every message boundary so each message starts with a clean buffer.
    void clear();

    // Append `n` samples at the producer cursor and advance it by n.
    // Allocation-free.
    void write(const float* in, size_t n);

    // Absolute source-sample count written so far since the last clear().
    uint64_t producer_position() const { return producer_position_; }

    // Physical ring capacity in samples (power of two).
    size_t capacity() const { return buffer_.size(); }

    // Catmull-Rom / Hermite cubic interpolation at fractional source-
    // sample position pos. See class-level comment for the safe range.
    float read_at(double pos) const;

private:
    std::vector<float> buffer_;
    size_t   mask_              = 0;
    uint64_t producer_position_ = 0;
};

} // namespace openCREST::dsp
