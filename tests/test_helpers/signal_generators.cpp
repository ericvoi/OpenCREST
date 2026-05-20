#include "test_helpers/signal_generators.hpp"
#include <cmath>

namespace openCREST::test {

static constexpr float TWO_PI = 6.28318530717958647693f;

std::vector<float> make_tone(float freq_hz, float amplitude,
                              float sample_rate, size_t num_samples) {
    std::vector<float> out(num_samples);
    for (size_t n = 0; n < num_samples; ++n) {
        out[n] = amplitude * std::sin(TWO_PI * freq_hz * static_cast<float>(n)
                                                        / sample_rate);
    }
    return out;
}

std::vector<float> make_chirp(float freq_start_hz, float freq_end_hz,
                               float amplitude, float sample_rate,
                               size_t num_samples) {
    std::vector<float> out(num_samples);
    const float T = static_cast<float>(num_samples) / sample_rate;
    for (size_t n = 0; n < num_samples; ++n) {
        const float t    = static_cast<float>(n) / sample_rate;
        const float freq = freq_start_hz + (freq_end_hz - freq_start_hz) * t / T;
        out[n] = amplitude * std::sin(TWO_PI * freq * t);
    }
    return out;
}

std::vector<float> make_impulse(size_t num_samples, size_t offset,
                                 float amplitude) {
    std::vector<float> out(num_samples, 0.0f);
    if (offset < num_samples) out[offset] = amplitude;
    return out;
}

std::vector<float> make_dc(float value, size_t num_samples) {
    return std::vector<float>(num_samples, value);
}

} // namespace openCREST::test
