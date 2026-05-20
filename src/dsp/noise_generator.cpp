#include "dsp/noise_generator.hpp"
#include "dsp/wenz_model.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <stdexcept>

namespace openCREST::dsp {

static constexpr float TWO_PI = 6.28318530717958647693f;

NoiseGenerator::NoiseGenerator(const NoiseConfig& config, uint32_t sample_rate) {
    init_shaping_filter(config, sample_rate);
    init_tonals(config, sample_rate);
    set_seed(12345);
}

void NoiseGenerator::init_shaping_filter(const NoiseConfig& config,
                                          uint32_t sample_rate) {
    shaping_coeffs_ = design_shaping_filter(
        static_cast<float>(sample_rate),
        config.wenz_sea_state,
        config.saltwater,
        32);

    const size_t M = shaping_coeffs_.size();
    filter_state_.assign(M - 1, 0.0f);

    // Compute the expected RMS of the filtered white noise:
    //   σ_out² = σ_in² · Σ h[k]²  (for unit-variance input, σ_in² = 1)
    float energy = 0.0f;
    for (float c : shaping_coeffs_) energy += c * c;
    const float sigma_out = (energy > 0.0f) ? std::sqrt(energy) : 1.0f;

    // Scale factor maps unit-variance shaped noise to the target dBFS level.
    //   target_rms = 10^(target_level_db_re_fs / 20)
    const float target_rms = std::pow(10.0f, config.target_level_db_re_fs / 20.0f);
    output_scale_ = target_rms / sigma_out;
}

void NoiseGenerator::init_tonals(const NoiseConfig& config, uint32_t sample_rate) {
    const float target_rms = std::pow(10.0f, config.target_level_db_re_fs / 20.0f);

    tonals_.reserve(config.tonals.size());
    for (const auto& t : config.tonals) {
        TonalOscillator osc{};
        osc.phase_inc  = TWO_PI * t.frequency_hz / static_cast<float>(sample_rate);
        osc.amplitude  = t.amplitude_linear * target_rms;
        if (t.bandwidth_hz > 0.0f) {
            osc.bw_phase_inc = TWO_PI * t.bandwidth_hz / static_cast<float>(sample_rate);
        }
        tonals_.push_back(osc);
    }
}

void NoiseGenerator::generate(float* output, size_t count) {
    const size_t M = shaping_coeffs_.size();

    for (size_t i = 0; i < count; ++i) {
        // --- Wenz-shaped noise ---
        // Push a new white-noise sample through the FIR filter.
        const float white = gaussian_(rng_);

        // Shift state right and insert new sample at position 0.
        // filter_state_ holds the last (M-1) white samples.
        float y = shaping_coeffs_[0] * white;
        for (size_t k = 1; k < M; ++k) {
            y += shaping_coeffs_[k] * ((k - 1 < filter_state_.size())
                                        ? filter_state_[k - 1]
                                        : 0.0f);
        }
        // Shift: oldest sample falls off the back
        for (size_t k = filter_state_.size(); k-- > 1; ) {
            filter_state_[k] = filter_state_[k - 1];
        }
        if (!filter_state_.empty()) filter_state_[0] = white;

        output[i] = y * output_scale_;

        // --- Tonal sources ---
        for (auto& osc : tonals_) {
            float amp = osc.amplitude;
            if (osc.bw_phase_inc > 0.0f) {
                // Narrowband: modulate amplitude with a cosine dither
                amp *= 0.5f * (1.0f + std::cos(osc.bw_phase));
                osc.bw_phase += osc.bw_phase_inc;
                if (osc.bw_phase > TWO_PI) osc.bw_phase -= TWO_PI;
            }
            output[i] += amp * std::cos(osc.phase);
            osc.phase += osc.phase_inc;
            if (osc.phase > TWO_PI) osc.phase -= TWO_PI;
        }
    }
}

void NoiseGenerator::set_seed(uint64_t seed) {
    rng_.seed(seed);
    gaussian_.reset();
}

void NoiseGenerator::reset() {
    std::fill(filter_state_.begin(), filter_state_.end(), 0.0f);
    for (auto& osc : tonals_) {
        osc.phase    = 0.0f;
        osc.bw_phase = 0.0f;
    }
    gaussian_.reset();
}

} // namespace openCREST::dsp
