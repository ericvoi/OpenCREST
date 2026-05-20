#pragma once
#include <array>
#include <cstddef>

namespace openCREST::dsp {

// Output-driven fractional sample-rate converter using 4-point cubic
// (Catmull-Rom / Hermite) interpolation.
//
// "Output-driven" means the caller specifies how many output samples to
// produce; the resampler consumes however many input samples are required.
//
// ratio is the per-output phase advance through the input stream — equivalently
// the number of input samples consumed per output sample produced. Because
// input and output share the same wall-clock sample rate, ratio is also the
// signal-frequency multiplier from input to output.
//
//   ratio = 1.0 → identity (no resampling; one input per output)
//   ratio > 1.0 → consume MORE inputs than outputs per block.
//                 Output signal compressed in time → frequency shifted UP.
//   ratio < 1.0 → consume FEWER inputs than outputs per block.
//                 Output signal stretched in time → frequency shifted DOWN.
//
// Doppler correction example:
//   Source moving toward receiver at v m/s in water (sound speed c m/s):
//   ratio ≈ 1 + v/c   (slightly above 1.0; receiver hears higher frequency)
//
// The resampler maintains a 4-sample history across process() calls so it
// produces a continuous stream when called repeatedly with consecutive blocks.
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
    // Output and input buffers must not alias.
    // May produce fewer than `output_count` if input is exhausted.
    Result process(const float* input, size_t input_available,
                   float*       output, size_t output_count);

    // Conservative upper bound on input samples needed to produce N outputs.
    size_t input_needed(size_t output_count) const;

    void reset();

private:
    double ratio_ = 1.0;
    double step_  = 1.0;   // = ratio_  (phase advance per output sample)
    double mu_    = 0.0;   // fractional phase in [0, 1)

    // Circular history of the last 4 input samples.
    // Interpolation is performed between history_[1] and history_[2].
    std::array<float, 4> history_ = {0.f, 0.f, 0.f, 0.f};

    // Hermite cubic interpolation at fractional position mu in [0,1)
    // using history_[0..3] as x[n-1], x[n], x[n+1], x[n+2].
    float interpolate() const;
};

} // namespace openCREST::dsp
