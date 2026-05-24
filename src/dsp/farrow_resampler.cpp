#include "dsp/farrow_resampler.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace openCREST::dsp {

FarrowResampler::FarrowResampler() = default;

void FarrowResampler::set_ratio(double ratio) {
    if (ratio <= 0.0) {
        throw std::invalid_argument("FarrowResampler: ratio must be > 0");
    }
    ratio_ = ratio;
    step_  = ratio;
}

// Catmull-Rom / Hermite cubic interpolation in Horner form.
// history_[0..3] = x[n-1], x[n], x[n+1], x[n+2]; mu_ ∈ [0,1).
//   y = x[n] + 0.5·t·{ (x[n+1]−x[n-1])
//                     + t·[ 2·x[n-1] − 5·x[n] + 4·x[n+1] − x[n+2]
//                          + t·(3·(x[n]−x[n+1]) + x[n+2] − x[n-1]) ] }
float FarrowResampler::interpolate() const {
    const float h0 = history_[0]; // x[n-1]
    const float h1 = history_[1]; // x[n]
    const float h2 = history_[2]; // x[n+1]
    const float h3 = history_[3]; // x[n+2]
    const float t  = static_cast<float>(mu_);

    return h1 + 0.5f * t * ((h2 - h0)
        + t * ((2.0f * h0 - 5.0f * h1 + 4.0f * h2 - h3)
            + t * (3.0f * (h1 - h2) + h3 - h0)));
}

FarrowResampler::Result
FarrowResampler::process(const float* input, size_t input_available,
                         float*       output, size_t output_count) {
    size_t produced = 0;
    size_t consumed = 0;

    while (produced < output_count) {
        // Advance history until mu_ < 1.0, consuming input as needed.
        while (mu_ >= 1.0) {
            if (consumed >= input_available) {
                return {produced, consumed};
            }
            history_[0] = history_[1];
            history_[1] = history_[2];
            history_[2] = history_[3];
            history_[3] = input[consumed++];
            mu_ -= 1.0;
        }

        output[produced++] = interpolate();
        mu_ += step_;
    }

    return {produced, consumed};
}

size_t FarrowResampler::input_needed(size_t output_count) const {
    // Conservative: (output_count × ratio) rounded up, plus 4 history samples.
    return static_cast<size_t>(std::ceil(static_cast<double>(output_count) * ratio_))
           + 4;
}

void FarrowResampler::reset() {
    mu_      = 0.0;
    history_ = {0.f, 0.f, 0.f, 0.f};
}

} // namespace openCREST::dsp
