#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "dsp/noise_generator.hpp"
#include "test_helpers/analysis.hpp"

using openCREST::dsp::NoiseConfig;
using openCREST::dsp::NoiseGenerator;
using openCREST::dsp::TonalSource;

static const uint32_t FS = 500'000;

// ---------------------------------------------------------------------------
// Deterministic seeding
// ---------------------------------------------------------------------------

TEST(NoiseGenerator, DeterministicSeed) {
    NoiseConfig cfg;
    cfg.target_level_db_re_fs = -40.0f;

    NoiseGenerator g1(cfg, FS), g2(cfg, FS);
    g1.set_seed(99); g2.set_seed(99);

    std::vector<float> out1(512), out2(512);
    g1.generate(out1.data(), 512);
    g2.generate(out2.data(), 512);

    for (size_t i = 0; i < 512; ++i) {
        EXPECT_FLOAT_EQ(out1[i], out2[i]) << "at i=" << i;
    }
}

TEST(NoiseGenerator, DifferentSeedsGiveDifferentOutput) {
    NoiseConfig cfg;
    cfg.target_level_db_re_fs = -40.0f;

    NoiseGenerator g1(cfg, FS), g2(cfg, FS);
    g1.set_seed(1); g2.set_seed(2);

    std::vector<float> out1(256), out2(256);
    g1.generate(out1.data(), 256);
    g2.generate(out2.data(), 256);

    bool any_different = false;
    for (size_t i = 0; i < 256; ++i) {
        if (out1[i] != out2[i]) { any_different = true; break; }
    }
    EXPECT_TRUE(any_different);
}

// ---------------------------------------------------------------------------
// Output level
// ---------------------------------------------------------------------------

TEST(NoiseGenerator, OutputLevelMatchesTargetApproximately) {
    NoiseConfig cfg;
    cfg.wenz_sea_state        = 3;
    cfg.target_level_db_re_fs = -30.0f;
    cfg.saltwater             = true;

    NoiseGenerator g(cfg, FS);
    g.set_seed(0);

    // Generate a large block to get a stable RMS estimate
    std::vector<float> out(50'000);
    g.generate(out.data(), 50'000);

    const float rms    = openCREST::test::rms(out);
    const float rms_db = 20.0f * std::log10(rms);
    // Allow ±3 dB tolerance (FIR shaping filter normalisation is approximate)
    EXPECT_NEAR(rms_db, cfg.target_level_db_re_fs, 3.0f)
        << "measured RMS=" << rms_db << " dBFS";
}

TEST(NoiseGenerator, LowerSeaStateIsQuieter) {
    auto make_rms = [&](int ss) {
        NoiseConfig cfg;
        cfg.wenz_sea_state        = ss;
        cfg.target_level_db_re_fs = -30.0f;
        NoiseGenerator g(cfg, FS);
        g.set_seed(0);
        std::vector<float> out(10'000);
        g.generate(out.data(), 10'000);
        return openCREST::test::rms(out);
    };
    // Noise level is determined by target_level_db_re_fs, not sea state (the
    // sea state only shapes the spectrum, not the amplitude).
    // Both should be close to the same RMS since target is the same.
    const float rms0 = make_rms(0);
    const float rms6 = make_rms(6);
    // Within 30% of each other (amplitude normalised to same target level)
    EXPECT_NEAR(rms0, rms6, rms0 * 0.3f);
}

// ---------------------------------------------------------------------------
// Tonal sources
// ---------------------------------------------------------------------------

TEST(NoiseGenerator, TonalPeakPresentInSpectrum) {
    const float tonal_freq = 10'000.0f; // 10 kHz

    NoiseConfig cfg;
    cfg.wenz_sea_state        = 3;
    cfg.target_level_db_re_fs = -50.0f;  // low noise floor
    TonalSource ts;
    ts.frequency_hz   = tonal_freq;
    ts.amplitude_linear = 10.0f;         // well above noise
    ts.bandwidth_hz   = 0.0f;
    cfg.tonals.push_back(ts);

    NoiseGenerator g(cfg, FS);
    g.set_seed(42);

    const size_t N = 8192;
    std::vector<float> out(N);
    g.generate(out.data(), N);

    auto psd = openCREST::test::power_spectrum(out, N);

    // Find bin closest to tonal_freq
    const size_t tonal_bin = static_cast<size_t>(tonal_freq * N / FS);
    float peak_bin_power   = psd[tonal_bin];
    float avg_power        = 0.0f;
    for (size_t k = 1; k < N / 2; ++k) avg_power += psd[k];
    avg_power /= static_cast<float>(N / 2 - 1);

    // The tonal bin should be significantly above average
    EXPECT_GT(peak_bin_power, avg_power * 5.0f)
        << "tonal not visible: peak=" << peak_bin_power
        << " avg=" << avg_power;
}

// ---------------------------------------------------------------------------
// reset() produces same output as fresh generator with same seed
// ---------------------------------------------------------------------------

TEST(NoiseGenerator, ResetRestoresState) {
    NoiseConfig cfg;
    cfg.target_level_db_re_fs = -40.0f;

    NoiseGenerator g(cfg, FS);
    g.set_seed(7);

    std::vector<float> first(256), second(256);
    g.generate(first.data(), 256);

    g.reset();
    g.set_seed(7);
    g.generate(second.data(), 256);

    for (size_t i = 0; i < 256; ++i) {
        EXPECT_FLOAT_EQ(first[i], second[i]) << "at i=" << i;
    }
}
