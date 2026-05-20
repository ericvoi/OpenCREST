#pragma once
#include <vector>
#include <random>
#include <cstdint>

namespace openCREST::dsp {

struct TonalSource {
    float frequency_hz;      // centre frequency
    float amplitude_linear;  // amplitude relative to ambient noise RMS
    float bandwidth_hz;      // 0 = pure tone, > 0 = narrowband
};

struct NoiseConfig {
    int   wenz_sea_state           = 3;      // 0 (calm) – 6 (very rough)
    float target_level_db_re_fs    = -40.0f; // dBFS, e.g. -40 dBFS
    bool  saltwater                = true;
    std::vector<TonalSource> tonals;
};

// Ambient noise generator.
//
// Produces Wenz-shaped (coloured) Gaussian noise scaled to a target level,
// plus optional tonal (narrowband) sources.
//
// Designed for per-receiver injection after channel summation. Deterministic
// seeding via set_seed() ensures reproducible test output.
class NoiseGenerator {
public:
    NoiseGenerator(const NoiseConfig& config, uint32_t sample_rate);

    // Fill `output` with `count` noise + tonal samples.
    void generate(float* output, size_t count);

    // Restart with a deterministic seed (useful for unit tests).
    void set_seed(uint64_t seed);

    void reset();

private:
    void init_shaping_filter(const NoiseConfig& config, uint32_t sample_rate);
    void init_tonals(const NoiseConfig& config, uint32_t sample_rate);

    // White-noise source
    std::mt19937_64                  rng_;
    std::normal_distribution<float>  gaussian_{0.0f, 1.0f};

    // Wenz shaping FIR (direct form)
    std::vector<float> shaping_coeffs_;
    std::vector<float> filter_state_;  // last (M-1) white-noise samples

    // Per-tonal oscillator
    struct TonalOscillator {
        float phase          = 0.0f;
        float phase_inc      = 0.0f;  // 2π·f / fs
        float amplitude      = 0.0f;  // absolute amplitude
        // Narrowband: amplitude is modulated by a low-rate noise process
        float bw_phase       = 0.0f;
        float bw_phase_inc   = 0.0f;  // 2π·bw / fs
    };
    std::vector<TonalOscillator> tonals_;

    float output_scale_ = 1.0f;  // converts unit-variance shaped noise → dBFS
};

} // namespace openCREST::dsp
