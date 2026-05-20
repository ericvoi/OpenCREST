#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "dsp/farrow_resampler.hpp"
#include "test_helpers/signal_generators.hpp"
#include "test_helpers/analysis.hpp"

using openCREST::dsp::FarrowResampler;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Run the resampler on `input`, collecting all produced samples.
static std::vector<float> resample_all(FarrowResampler& r,
                                       const std::vector<float>& input,
                                       size_t output_count) {
    std::vector<float> out(output_count, 0.0f);
    r.process(input.data(), input.size(), out.data(), output_count);
    return out;
}

// ---------------------------------------------------------------------------
// ratio = 1.0 — identity
// ---------------------------------------------------------------------------

TEST(FarrowResampler, RatioOneIsIdentity) {
    FarrowResampler r;
    r.set_ratio(1.0);

    // Feed a DC signal; output (after warm-up) should equal the input.
    const size_t N = 512;
    std::vector<float> input(N, 0.5f);
    std::vector<float> output(N, 0.0f);

    r.process(input.data(), N, output.data(), N);

    // Skip first 4 samples (interpolator warm-up) then compare
    for (size_t i = 4; i < N; ++i) {
        EXPECT_NEAR(output[i], 0.5f, 1e-4f) << "at i=" << i;
    }
}

TEST(FarrowResampler, RatioOneSinePassthrough) {
    FarrowResampler r;
    r.set_ratio(1.0);

    const float fs   = 500'000.0f;
    const float freq = 10'000.0f;
    auto input = openCREST::test::make_tone(freq, 0.8f, fs, 2048);
    std::vector<float> output(2048, 0.0f);

    r.process(input.data(), input.size(), output.data(), 2048);

    // SNR should be very high (interpolation error only)
    // 3-sample latency: output[n] == input[n-3] for n >= 3
    std::vector<float> ref(input.begin(),      input.end() - 3);  // input[0..2044]
    std::vector<float> got(output.begin() + 3, output.end());     // output[3..2047]
    const float snr = openCREST::test::snr_db(ref, got);
    EXPECT_GT(snr, 60.0f) << "SNR=" << snr << " dB";
}

// ---------------------------------------------------------------------------
// Upsampling (ratio > 1): output count > input count
// ---------------------------------------------------------------------------

TEST(FarrowResampler, UpsamplingByTwo) {
    FarrowResampler r;
    r.set_ratio(0.5);  // 0.5 inputs per output → 2 outputs per input

    const size_t N_in  = 256;
    const size_t N_out = 512;

    std::vector<float> input(N_in, 1.0f); // DC
    std::vector<float> output(N_out, 0.0f);

    auto result = r.process(input.data(), N_in, output.data(), N_out);

    // With DC input, all output should approach 1.0 after warm-up
    EXPECT_EQ(result.output_produced, N_out);
    for (size_t i = 8; i < N_out; ++i) {
        EXPECT_NEAR(output[i], 1.0f, 1e-3f) << "at i=" << i;
    }
}

// ---------------------------------------------------------------------------
// Downsampling (ratio < 1): fewer outputs than inputs
// ---------------------------------------------------------------------------

TEST(FarrowResampler, DownsamplingByTwo) {
    FarrowResampler r;
    r.set_ratio(2.0);  // 2 inputs per output → 1 output per 2 inputs

    const size_t N_in  = 512;
    const size_t N_out = 256;

    std::vector<float> input(N_in, 1.0f); // DC
    std::vector<float> output(N_out, 0.0f);

    auto result = r.process(input.data(), N_in, output.data(), N_out);

    EXPECT_EQ(result.output_produced, N_out);
    for (size_t i = 4; i < N_out; ++i) {
        EXPECT_NEAR(output[i], 1.0f, 1e-3f) << "at i=" << i;
    }
}

// ---------------------------------------------------------------------------
// Frequency shift from Doppler ratio
// ---------------------------------------------------------------------------

TEST(FarrowResampler, DopplerShiftsToneFrequency) {
    // A 10 kHz tone with ratio 1.05 should appear at ~10.5 kHz in the output
    const float fs   = 500'000.0f;
    const float fin  = 10'000.0f;
    const double ratio = 1.05;
    const float fout_expected = static_cast<float>(fin * ratio);

    FarrowResampler r;
    r.set_ratio(ratio);

    const size_t N_in  = 4096;
    const size_t N_out = static_cast<size_t>(static_cast<double>(N_in) / ratio);

    auto input = openCREST::test::make_tone(fin, 0.5f, fs, N_in);
    std::vector<float> output(N_out, 0.0f);

    r.process(input.data(), N_in, output.data(), N_out);

    // Zero-crossing frequency estimate (skip warm-up)
    std::vector<float> tail(output.begin() + 64, output.end());
    const float fmeas = openCREST::test::estimate_frequency_hz(tail, fs);
    EXPECT_NEAR(fmeas, fout_expected, fout_expected * 0.02f)  // 2% tolerance
        << "measured=" << fmeas << " expected=" << fout_expected;
}

// ---------------------------------------------------------------------------
// input_needed estimate is conservative
// ---------------------------------------------------------------------------

TEST(FarrowResampler, InputNeededIsConservative) {
    FarrowResampler r;
    r.set_ratio(1.0);
    EXPECT_GE(r.input_needed(256), 256u);

    r.set_ratio(2.0);
    EXPECT_GE(r.input_needed(256), 512u);  // 2 inputs/output: need ~512 inputs for 256 outputs

    r.set_ratio(0.5);
    EXPECT_GE(r.input_needed(256), 128u);  // 0.5 inputs/output: need ~128 inputs for 256 outputs
}

// ---------------------------------------------------------------------------
// reset() clears state
// ---------------------------------------------------------------------------

TEST(FarrowResampler, ResetClearsHistory) {
    FarrowResampler r;
    r.set_ratio(1.0);

    std::vector<float> ones(256, 1.0f);
    std::vector<float> out(256);
    r.process(ones.data(), 256, out.data(), 256);

    r.reset();

    // After reset, warm-up outputs should again be zero (no old history)
    std::vector<float> zeros(256, 0.0f);
    r.process(zeros.data(), 256, out.data(), 256);
    for (float v : out) EXPECT_FLOAT_EQ(v, 0.0f);
}
