#include "test_helpers/analysis.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <stdexcept>

namespace openCREST::test {

static constexpr float TWO_PI = 6.28318530717958647693f;

float rms(const std::vector<float>& x) {
    if (x.empty()) return 0.0f;
    float sum = 0.0f;
    for (float v : x) sum += v * v;
    return std::sqrt(sum / static_cast<float>(x.size()));
}

float snr_db(const std::vector<float>& ref, const std::vector<float>& test) {
    if (ref.size() != test.size()) {
        throw std::invalid_argument("snr_db: ref and test must have the same length");
    }
    float signal_power = 0.0f;
    float noise_power  = 0.0f;
    for (size_t i = 0; i < ref.size(); ++i) {
        signal_power += ref[i] * ref[i];
        float e = ref[i] - test[i];
        noise_power += e * e;
    }
    if (noise_power == 0.0f) return 200.0f; // effectively infinite
    return 10.0f * std::log10(signal_power / noise_power);
}

float peak(const std::vector<float>& x) {
    if (x.empty()) return 0.0f;
    float m = 0.0f;
    for (float v : x) m = std::max(m, std::abs(v));
    return m;
}

float estimate_frequency_hz(const std::vector<float>& x, float sample_rate) {
    // Count zero crossings (positive slope)
    size_t crossings = 0;
    for (size_t i = 1; i < x.size(); ++i) {
        if (x[i - 1] < 0.0f && x[i] >= 0.0f) ++crossings;
    }
    // Each positive crossing = half cycle
    if (crossings == 0) return 0.0f;
    const float duration_s = static_cast<float>(x.size()) / sample_rate;
    return static_cast<float>(crossings) / duration_s;
}

int estimate_delay_samples(const std::vector<float>& ref,
                           const std::vector<float>& delayed,
                           int max_lag) {
    int best_lag = 0;
    float best_corr = -1e30f;

    for (int lag = -max_lag; lag <= max_lag; ++lag) {
        float corr = 0.0f;
        for (size_t i = 0; i < ref.size(); ++i) {
            const int j = static_cast<int>(i) - lag;
            if (j >= 0 && j < static_cast<int>(delayed.size())) {
                corr += ref[i] * delayed[j];
            }
        }
        if (corr > best_corr) {
            best_corr = corr;
            best_lag  = lag;
        }
    }
    return best_lag;
}

std::vector<float> power_spectrum(const std::vector<float>& x, size_t num_bins) {
    const size_t N = num_bins;
    std::vector<float> psd(N, 0.0f);
    for (size_t k = 0; k < N; ++k) {
        float re = 0.0f, im = 0.0f;
        for (size_t n = 0; n < x.size(); ++n) {
            const float angle = -TWO_PI * static_cast<float>(k * n)
                                        / static_cast<float>(x.size());
            re += x[n] * std::cos(angle);
            im += x[n] * std::sin(angle);
        }
        psd[k] = (re * re + im * im) / static_cast<float>(x.size());
    }
    return psd;
}

} // namespace openCREST::test
