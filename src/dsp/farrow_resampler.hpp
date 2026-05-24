#pragma once
#include <array>
#include <cstddef>

namespace openCREST::dsp {

// Output-driven fractional sample-rate converter using 4-point cubic
// (Catmull-Rom / Hermite) interpolation.
//
// The caller specifies how many output samples to produce; the resampler
// consumes whatever input is needed.
//
// ratio = per-output phase advance through the input stream = input
// samples consumed per output sample. Input and output share the same
// wall-clock sample rate, so ratio is also the signal-frequency
// multiplier from input to output:
//   ratio = 1.0 → identity.
//   ratio > 1.0 → consume MORE inputs than outputs (frequency shifted UP).
//   ratio < 1.0 → consume FEWER inputs than outputs (frequency shifted DOWN).
//
// Doppler example: source approaching receiver at v m/s in water (sound
// speed c) → ratio ≈ 1 + v/c.
//
// A 4-sample history persists across process() calls, so consecutive
// blocks produce a continuous output stream.
class FarrowResampler {
public:
    FarrowResampler();

    void   set_ratio(double ratio);
    double ratio() const { return ratio_; }

    struct Result {
        size_t output_produced;
        size_t input_consumed;
    };

    // Produce up to `output_count` samples using input[0..input_available-1].
    // Output and input buffers must not alias. Produces fewer than
    // `output_count` only when input is exhausted.
    Result process(const float* input, size_t input_available,
                   float*       output, size_t output_count);

    // Conservative upper bound on input samples needed to produce N outputs.
    size_t input_needed(size_t output_count) const;

    void reset();

private:
    double ratio_ = 1.0;
    double step_  = 1.0;   // = ratio_  (phase advance per output sample)
    double mu_    = 0.0;   // fractional phase in [0, 1)

    // Last 4 input samples; interpolation runs between history_[1] and history_[2].
    std::array<float, 4> history_ = {0.f, 0.f, 0.f, 0.f};

    float interpolate() const;
};

} // namespace openCREST::dsp
