#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "dsp/hilbert_filter.hpp"
#include "test_helpers/signal_generators.hpp"

using openCREST::dsp::HilbertFilter;

namespace {

constexpr float FS = 500'000.0f;

void process_block(HilbertFilter& f,
                   const std::vector<float>& in,
                   std::vector<float>& delayed_real,
                   std::vector<float>& hilbert_out) {
    delayed_real.assign(in.size(), 0.0f);
    hilbert_out .assign(in.size(), 0.0f);
    f.process(in.data(), delayed_real.data(), hilbert_out.data(), in.size());
}

} // namespace

// ===========================================================================
// Construction-time invariants
// ===========================================================================

TEST(HilbertFilter, EvenLengthRejected) {
    EXPECT_THROW(HilbertFilter(30), std::invalid_argument);
}

TEST(HilbertFilter, GroupDelayIsHalfLength) {
    HilbertFilter f31(31);
    EXPECT_EQ(f31.group_delay_samples(), 15u);
    HilbertFilter f63(63);
    EXPECT_EQ(f63.group_delay_samples(), 31u);
}

// ===========================================================================
// Zero in → zero out
// ===========================================================================

TEST(HilbertFilter, ZeroInputProducesZeroOutput) {
    HilbertFilter f(31);
    std::vector<float> in(256, 0.0f);
    std::vector<float> dr, ho;
    process_block(f, in, dr, ho);
    for (float x : dr) EXPECT_FLOAT_EQ(x, 0.0f);
    for (float x : ho) EXPECT_FLOAT_EQ(x, 0.0f);
}

// ===========================================================================
// Delayed-real path: delay equals group_delay_samples()
// ===========================================================================

TEST(HilbertFilter, DelayedRealMatchesInputAfterGroupDelay) {
    HilbertFilter f(31);
    constexpr size_t N = 128;
    std::vector<float> in(N);
    for (size_t i = 0; i < N; ++i) in[i] = std::sin(0.05f * i);

    std::vector<float> dr, ho;
    process_block(f, in, dr, ho);

    const size_t D = f.group_delay_samples();
    for (size_t i = D; i < N; ++i) {
        EXPECT_NEAR(dr[i], in[i - D], 1e-6f) << "i=" << i;
    }
    // Pre-delay region is zero (history starts at 0).
    for (size_t i = 0; i < D; ++i) {
        EXPECT_FLOAT_EQ(dr[i], 0.0f);
    }
}

// ===========================================================================
// 90° phase shift on a mid-band tone: cos → sin
// ===========================================================================

TEST(HilbertFilter, MidBandToneIsShifted90Degrees) {
    // 30 kHz tone at 500 kSPS → fc/Fs = 0.06 → well inside the mid-band.
    constexpr float FREQ = 30'000.0f;
    constexpr size_t N   = 4096;

    HilbertFilter f(31);
    auto cos_input = std::vector<float>(N);
    for (size_t i = 0; i < N; ++i) {
        cos_input[i] = std::cos(2.0f * M_PI * FREQ * i / FS);
    }

    std::vector<float> dr, ho;
    process_block(f, cos_input, dr, ho);

    // After group delay, dr should track cos(ω·n − ω·D) and ho should track
    // sin(ω·n − ω·D). Compare only the steady-state region (skip warmup).
    const size_t D     = f.group_delay_samples();
    const size_t start = D + 64;
    const size_t end   = N - 64;

    double sum_cos2 = 0.0, sum_sin2 = 0.0;
    double sum_cross = 0.0;
    for (size_t i = start; i < end; ++i) {
        const float ref_cos = std::cos(
            2.0f * M_PI * FREQ * (i - D) / FS);
        const float ref_sin = std::sin(
            2.0f * M_PI * FREQ * (i - D) / FS);
        sum_cos2  += static_cast<double>(ref_cos) * dr[i];
        sum_sin2  += static_cast<double>(ref_sin) * ho[i];
        sum_cross += static_cast<double>(ref_cos) * ho[i];
    }
    const double n_avg = static_cast<double>(end - start);
    // dr correlates strongly with the delayed cosine (≥ 0.5 — Hamming
    // window introduces a mid-band gain near unity but slightly less).
    EXPECT_GT(sum_cos2 / n_avg, 0.4);
    // ho correlates strongly with the delayed sine.
    EXPECT_GT(sum_sin2 / n_avg, 0.4);
    // ho should be uncorrelated with the delayed cosine (Hilbert pair).
    EXPECT_LT(std::abs(sum_cross / n_avg), 0.05);
}

// ===========================================================================
// Block-boundary consistency: chunked vs single-shot processing
// ===========================================================================

TEST(HilbertFilter, ChunkedProcessingMatchesSingleShot) {
    constexpr size_t N = 1024;
    HilbertFilter f1(31), f2(31);

    std::vector<float> in(N);
    for (size_t i = 0; i < N; ++i) {
        in[i] = std::cos(0.07f * i) + 0.3f * std::sin(0.13f * i);
    }

    std::vector<float> dr1, ho1;
    process_block(f1, in, dr1, ho1);

    // Now process via 64-sample chunks with f2.
    std::vector<float> dr2(N, 0.0f), ho2(N, 0.0f);
    constexpr size_t CHUNK = 64;
    for (size_t i = 0; i < N; i += CHUNK) {
        const size_t take = std::min(CHUNK, N - i);
        f2.process(in.data() + i, dr2.data() + i, ho2.data() + i, take);
    }

    for (size_t i = 0; i < N; ++i) {
        EXPECT_NEAR(dr1[i], dr2[i], 1e-6f) << "delayed_real diff at i=" << i;
        EXPECT_NEAR(ho1[i], ho2[i], 1e-6f) << "hilbert diff at i=" << i;
    }
}

// ===========================================================================
// reset() restores initial state
// ===========================================================================

TEST(HilbertFilter, ResetRestoresFreshState) {
    HilbertFilter f(31);
    std::vector<float> in(256);
    for (size_t i = 0; i < 256; ++i) in[i] = static_cast<float>(i % 17 - 8);
    std::vector<float> dr, ho;
    process_block(f, in, dr, ho);

    f.reset();

    // After reset, processing zero input should yield zero output.
    std::vector<float> z(256, 0.0f);
    std::vector<float> dr2, ho2;
    process_block(f, z, dr2, ho2);
    for (float x : dr2) EXPECT_FLOAT_EQ(x, 0.0f);
    for (float x : ho2) EXPECT_FLOAT_EQ(x, 0.0f);
}
