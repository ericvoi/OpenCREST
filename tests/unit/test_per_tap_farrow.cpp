#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "dsp/per_tap_farrow.hpp"
#include "dsp/source_delay_line.hpp"
#include "test_helpers/analysis.hpp"
#include "test_helpers/signal_generators.hpp"

using openCREST::dsp::PerTapFarrow;
using openCREST::dsp::SourceDelayLine;
using openCREST::dsp::TapState;

namespace {

constexpr float FS = 500'000.0f;

// Locate the index of the loudest spectral bin in [k_lo, k_hi).
size_t peak_bin(const std::vector<float>& psd, size_t k_lo, size_t k_hi) {
    size_t best = k_lo;
    float  best_v = psd[k_lo];
    for (size_t k = k_lo + 1; k < k_hi; ++k) {
        if (psd[k] > best_v) { best_v = psd[k]; best = k; }
    }
    return best;
}

// Convert bin index → Hz.
float bin_to_hz(size_t k, size_t N, float fs) {
    return static_cast<float>(k) * fs / static_cast<float>(N);
}

} // namespace

// ---------------------------------------------------------------------------
// n_out == 0 → no-op
// ---------------------------------------------------------------------------

TEST(PerTapFarrow, ZeroOutNoOp) {
    SourceDelayLine sdl;
    sdl.resize(64);

    TapState tap{};
    std::vector<float> out(8, 99.0f);
    PerTapFarrow::produce(sdl, tap, /*out_pos_start=*/0.0,
                          /*n_out=*/0, out.data());
    // Untouched.
    for (float v : out) EXPECT_FLOAT_EQ(v, 99.0f);
}

// ---------------------------------------------------------------------------
// Static delay + DC source → DC output (modulo amp)
// ---------------------------------------------------------------------------

TEST(PerTapFarrow, StaticDelayPassesDcThrough) {
    SourceDelayLine sdl;
    sdl.resize(1024);
    std::vector<float> dc(512, 0.7f);
    sdl.write(dc.data(), dc.size());

    TapState tap{};
    tap.tap_delay_samples_at_block_start = 10.0;
    tap.tap_delay_samples_at_block_end   = 10.0;
    tap.amplitude_at_block_start         = 1.0f;
    tap.amplitude_at_block_end           = 1.0f;

    std::vector<float> out(64, 0.0f);
    PerTapFarrow::produce(sdl, tap, /*out_pos_start=*/100.0,
                          /*n_out=*/64, out.data());
    // Catmull-Rom with constant input → output equals the constant.
    for (float v : out) EXPECT_NEAR(v, 0.7f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Amplitude lerp
// ---------------------------------------------------------------------------

TEST(PerTapFarrow, AmplitudeLerpDecaysLinearly) {
    SourceDelayLine sdl;
    sdl.resize(1024);
    std::vector<float> dc(512, 1.0f);
    sdl.write(dc.data(), dc.size());

    TapState tap{};
    tap.tap_delay_samples_at_block_start = 10.0;
    tap.tap_delay_samples_at_block_end   = 10.0;
    tap.amplitude_at_block_start         = 1.0f;
    tap.amplitude_at_block_end           = 0.0f;

    const size_t N = 100;
    std::vector<float> out(N, 0.0f);
    PerTapFarrow::produce(sdl, tap, /*out_pos_start=*/200.0,
                          N, out.data());

    // Linear from 1.0 down toward 0.0.
    for (size_t i = 0; i < N; ++i) {
        const float expected = 1.0f - static_cast<float>(i)
                                       / static_cast<float>(N);
        EXPECT_NEAR(out[i], expected, 2e-3f) << "i=" << i;
    }
}

// ---------------------------------------------------------------------------
// Linearly-evolving delay → frequency shift
// ---------------------------------------------------------------------------

TEST(PerTapFarrow, EvolvingDelayProducesFrequencyShift) {
    // dτ/dt > 0 stretches time → output frequency = source × (1 − dτ/dt).
    // Configure (delay_end − delay_start) / n_out so the output's bin peak
    // shifts ~300 Hz down from 30 kHz.

    SourceDelayLine sdl;
    sdl.resize(16384);

    constexpr float SRC_FREQ = 30'000.0f;
    constexpr size_t N_IN = 8192;
    auto tone = openCREST::test::make_tone(SRC_FREQ, 1.0f, FS, N_IN);
    sdl.write(tone.data(), tone.size());

    constexpr size_t N_OUT = 4096;
    // 300 Hz shift → dτ/dt = 300 / 30000 = 0.01.
    // Over N_OUT output samples, delay grows by 0.01 × N_OUT = 40.96.
    const double delta_delay = 0.01 * static_cast<double>(N_OUT);

    TapState tap{};
    tap.tap_delay_samples_at_block_start = 0.0;
    tap.tap_delay_samples_at_block_end   = delta_delay;
    tap.amplitude_at_block_start         = 1.0f;
    tap.amplitude_at_block_end           = 1.0f;

    std::vector<float> out(N_OUT, 0.0f);
    PerTapFarrow::produce(sdl, tap,
                          /*out_pos_start=*/100.0,
                          N_OUT, out.data());

    const auto psd = openCREST::test::power_spectrum(out, N_OUT);
    // Search around 30 kHz ± a few hundred Hz for the peak.
    const size_t k_lo = static_cast<size_t>(28'000.0f * N_OUT / FS);
    const size_t k_hi = static_cast<size_t>(32'000.0f * N_OUT / FS);
    const size_t kp   = peak_bin(psd, k_lo, k_hi);
    const float  fp   = bin_to_hz(kp, N_OUT, FS);

    // Expected shifted output frequency.
    const float expected_hz = 30'000.0f * (1.0f - 0.01f);
    // Bin resolution is FS / N_OUT ≈ 122 Hz; allow ±2 bins.
    EXPECT_NEAR(fp, expected_hz, 2.5f * FS / N_OUT)
        << "fp=" << fp << " expected=" << expected_hz;
}

TEST(PerTapFarrow, NoDriftWhenDelayStatic) {
    // dτ/dt = 0 → output frequency = source frequency exactly.
    SourceDelayLine sdl;
    sdl.resize(16384);
    auto tone = openCREST::test::make_tone(30'000.0f, 1.0f, FS, 8192);
    sdl.write(tone.data(), tone.size());

    TapState tap{};
    tap.tap_delay_samples_at_block_start = 5.0;
    tap.tap_delay_samples_at_block_end   = 5.0;
    tap.amplitude_at_block_start         = 1.0f;
    tap.amplitude_at_block_end           = 1.0f;

    const size_t N = 4096;
    std::vector<float> out(N, 0.0f);
    PerTapFarrow::produce(sdl, tap, /*out_pos_start=*/200.0, N, out.data());

    const auto psd = openCREST::test::power_spectrum(out, N);
    const size_t k_lo = static_cast<size_t>(28'000.0f * N / FS);
    const size_t k_hi = static_cast<size_t>(32'000.0f * N / FS);
    const size_t kp   = peak_bin(psd, k_lo, k_hi);
    const float  fp   = bin_to_hz(kp, N, FS);

    EXPECT_NEAR(fp, 30'000.0f, 2.0f * FS / N);
}

// ---------------------------------------------------------------------------
// Two taps with distinct Dopplers, additive when summed
// ---------------------------------------------------------------------------

TEST(PerTapFarrow, TwoTapsDistinctDopplersProduceTwoPeaks) {
    SourceDelayLine sdl;
    sdl.resize(16384);
    constexpr float SRC = 30'000.0f;
    auto tone = openCREST::test::make_tone(SRC, 1.0f, FS, 8192);
    sdl.write(tone.data(), tone.size());

    constexpr size_t N = 4096;

    // Tap A: dτ/dt = +0.01 → -300 Hz shift (29.7 kHz).
    // Tap B: dτ/dt = -0.005 → +150 Hz shift (30.15 kHz).
    TapState a{};
    a.tap_delay_samples_at_block_start = 0.0;
    a.tap_delay_samples_at_block_end   = 0.01 * static_cast<double>(N);
    a.amplitude_at_block_start         = 1.0f;
    a.amplitude_at_block_end           = 1.0f;

    TapState b{};
    b.tap_delay_samples_at_block_start = 100.0;
    b.tap_delay_samples_at_block_end   = 100.0 - 0.005 * static_cast<double>(N);
    b.amplitude_at_block_start         = 1.0f;
    b.amplitude_at_block_end           = 1.0f;

    std::vector<float> out_a(N, 0.0f);
    std::vector<float> out_b(N, 0.0f);
    PerTapFarrow::produce(sdl, a, /*out_pos_start=*/200.0, N, out_a.data());
    PerTapFarrow::produce(sdl, b, /*out_pos_start=*/200.0, N, out_b.data());

    // Sum additively.
    std::vector<float> sum(N);
    for (size_t i = 0; i < N; ++i) sum[i] = out_a[i] + out_b[i];

    const auto psd = openCREST::test::power_spectrum(sum, N);
    const float bin_hz = FS / static_cast<float>(N);

    // Locate the two strongest bins within 29 .. 31 kHz separated by
    // at least 200 Hz.
    const size_t k_lo = static_cast<size_t>(28'500.0f / bin_hz);
    const size_t k_hi = static_cast<size_t>(31'500.0f / bin_hz);

    size_t k1 = peak_bin(psd, k_lo, k_hi);

    // Mask out a ±2-bin neighbourhood around the first peak and find the second.
    auto psd2 = psd;
    for (size_t k = (k1 >= 2 ? k1 - 2 : 0); k <= k1 + 2 && k < psd2.size(); ++k) {
        psd2[k] = 0.0f;
    }
    size_t k2 = peak_bin(psd2, k_lo, k_hi);

    const float f1 = bin_to_hz(std::min(k1, k2), N, FS);
    const float f2 = bin_to_hz(std::max(k1, k2), N, FS);

    EXPECT_NEAR(f1, 29'700.0f, 2.0f * bin_hz);
    EXPECT_NEAR(f2, 30'150.0f, 2.0f * bin_hz);
}
