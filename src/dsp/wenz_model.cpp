#include "dsp/wenz_model.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace openCREST::dsp {

static constexpr float PI = 3.14159265358979323846f;

float sea_state_to_wind_knots(int sea_state) {
    // Beaufort → wind (knots), saturated at ss=6.
    static constexpr float table[] = {0.1f, 3.0f, 8.0f, 13.0f, 18.0f, 24.0f, 30.0f};
    const int idx = std::clamp(sea_state, 0, 6);
    return table[idx];
}

float wind_noise_psd_db(float freq_hz, int sea_state) {
    // Floor at 1 Hz to avoid log(0).
    const float f_khz = std::max(freq_hz, 1.0f) * 1e-3f;
    const float w     = sea_state_to_wind_knots(sea_state);
    return 50.0f + 7.5f * std::sqrt(w) + 20.0f * std::log10(f_khz)
                                        - 40.0f * std::log10(f_khz + 0.4f);
}

float thermal_noise_psd_db(float freq_hz) {
    const float f_khz = std::max(freq_hz, 1.0f) * 1e-3f;
    return -15.0f + 20.0f * std::log10(f_khz);
}

float noise_psd_db(float freq_hz, int sea_state, bool /*saltwater*/) {
    // Incoherent (power) sum of wind and thermal components. Saltwater
    // vs freshwater has a negligible effect on the thermal term for
    // filter-design purposes.
    const float p_wind    = std::pow(10.0f, wind_noise_psd_db(freq_hz, sea_state)    / 10.0f);
    const float p_thermal = std::pow(10.0f, thermal_noise_psd_db(freq_hz) / 10.0f);
    return 10.0f * std::log10(p_wind + p_thermal);
}

std::vector<float> design_shaping_filter(float sample_rate_hz,
                                         int   sea_state,
                                         bool  saltwater,
                                         size_t n_taps) {
    const size_t N      = std::max(n_taps, size_t{4});
    const size_t center = N / 2;

    // Step 1: desired amplitude response at N/2+1 unique frequencies.
    // H[k] = √(normalised PSD) — coloring filter for white-noise input.
    std::vector<float> H(N / 2 + 1);
    for (size_t k = 0; k <= N / 2; ++k) {
        float freq_hz = (k == 0) ? 1.0f  // avoid log(0) at DC
                                 : static_cast<float>(k) / static_cast<float>(N)
                                     * sample_rate_hz;
        float psd_db = noise_psd_db(freq_hz, sea_state, saltwater);
        H[k] = std::sqrt(std::pow(10.0f, psd_db / 10.0f));
    }

    // Normalise so peak amplitude = 1.0.
    float max_H = *std::max_element(H.begin(), H.end());
    if (max_H > 0.0f) {
        for (auto& h : H) h /= max_H;
    }

    // Step 2: symmetric IDFT (real & even spectrum → real & even h[n]).
    // Cosine-only form valid when H[-k] = H[k]:
    //   h[n] = (1/N) · [ H[0] + 2·Σ_{k=1}^{N/2-1} H[k]·cos(2πk(n−center)/N)
    //                          + H[N/2]·cos(π(n−center)) ]
    std::vector<float> h(N, 0.0f);
    for (size_t n = 0; n < N; ++n) {
        float val   = H[0];
        int   delta = static_cast<int>(n) - static_cast<int>(center);

        for (size_t k = 1; k < N / 2; ++k) {
            val += 2.0f * H[k]
                       * std::cos(2.0f * PI * static_cast<float>(k)
                                           * static_cast<float>(delta)
                                           / static_cast<float>(N));
        }
        // Nyquist bin (k = N/2).
        val += H[N / 2] * std::cos(PI * static_cast<float>(delta));

        h[n] = val / static_cast<float>(N);
    }

    // Step 3: Hann window to reduce spectral leakage.
    for (size_t n = 0; n < N; ++n) {
        float w = 0.5f * (1.0f - std::cos(2.0f * PI * static_cast<float>(n)
                                                     / static_cast<float>(N - 1)));
        h[n] *= w;
    }

    return h;
}

float filter_response_at(const std::vector<float>& taps,
                         float fc_hz, float fs_hz) {
    if (taps.empty() || fs_hz <= 0.0f) return 0.0f;
    const float omega = 2.0f * PI * fc_hz / fs_hz;
    float re = 0.0f, im = 0.0f;
    for (size_t n = 0; n < taps.size(); ++n) {
        const float arg = omega * static_cast<float>(n);
        re += taps[n] * std::cos(arg);
        im -= taps[n] * std::sin(arg);
    }
    return std::sqrt(re * re + im * im);
}

} // namespace openCREST::dsp
