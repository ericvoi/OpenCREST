#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "dsp/noise_generator.hpp"
#include "dsp/noise_psd.hpp"
#include "dsp/wenz_model.hpp"

using namespace openCREST::dsp;

namespace {

// Welch-averaged amplitude-PSD at a single frequency. Splits `signal` into
// `n_segments` of length `segment_len`, computes a single-bin DFT at `fc`
// per segment, and returns the mean one-sided amplitude PSD in dBFS/√Hz.
//
// PSD estimator details:
//   For a real signal, the one-sided power PSD at frequency f, estimated
//   from a length-N segment, is
//     S_one_sided(f) ≈ 2 · |X(f)|² / (N · Fs)
//   where X(f) = Σ x[n] · exp(−j2πfn/Fs).
//   Amplitude PSD = √S; in dB = 10·log10(S).
double measured_amp_psd_dbfs(const std::vector<float>& signal,
                              double fc, double fs,
                              size_t segment_len, size_t n_segments) {
    double psd_sum = 0.0;
    for (size_t s = 0; s < n_segments; ++s) {
        const float* seg = signal.data() + s * segment_len;
        double re = 0.0, im = 0.0;
        for (size_t n = 0; n < segment_len; ++n) {
            const double phase = 2.0 * M_PI * fc * static_cast<double>(n) / fs;
            re += seg[n] * std::cos(phase);
            im -= seg[n] * std::sin(phase);
        }
        const double mag_sq = re * re + im * im;
        const double psd_one_sided =
            2.0 * mag_sq / (static_cast<double>(segment_len) * fs);
        psd_sum += psd_one_sided;
    }
    const double avg_psd = psd_sum / static_cast<double>(n_segments);
    return 10.0 * std::log10(avg_psd);
}

} // namespace

// ===========================================================================
// Empirical: NoiseGenerator output PSD at fc matches the configured target
// PSD that psd_target_to_total_rms_dbfs derived. This is the load-bearing
// invariant for Phase C — if it drifts, Wenz/AFE comparisons in
// simulator.cpp produce nonsensical boost values.
// ===========================================================================

TEST(NoiseLevels, OutputPsdAtFcMatchesTarget) {
    constexpr float fs = 500'000.0f;
    constexpr float fc = 31'500.0f;
    constexpr int   ss = 3;
    constexpr float target_psd_dbfs = -80.0f;

    const auto taps = design_shaping_filter(fs, ss, /*saltwater=*/true, 32);
    const float total_rms_dbfs = psd_target_to_total_rms_dbfs(
        target_psd_dbfs, taps, fc, fs);
    ASSERT_TRUE(std::isfinite(total_rms_dbfs));

    NoiseConfig cfg;
    cfg.wenz_sea_state        = ss;
    cfg.target_level_db_re_fs = total_rms_dbfs;
    cfg.saltwater             = true;

    NoiseGenerator gen(cfg, static_cast<uint32_t>(fs));
    gen.set_seed(20260428);

    constexpr size_t segment_len = 8192;
    constexpr size_t n_segments  = 200;
    const size_t total = segment_len * n_segments;
    std::vector<float> out(total);
    gen.generate(out.data(), total);

    const double measured = measured_amp_psd_dbfs(
        out, fc, fs, segment_len, n_segments);

    // 200 segments → ≈ 7% standard error in the mean (≈ 0.6 dB).
    EXPECT_NEAR(measured, target_psd_dbfs, 1.5);
}

TEST(NoiseLevels, OutputPsdAtFcSurvivesTwoTargets) {
    // Repeat at a different target to verify the relationship is linear (a
    // 20 dB target shift should produce a 20 dB measured shift).
    constexpr float fs = 500'000.0f;
    constexpr float fc = 31'500.0f;
    constexpr int   ss = 3;
    const auto taps = design_shaping_filter(fs, ss, true, 32);

    auto run = [&](float target_dbfs) {
        const float rms_dbfs = psd_target_to_total_rms_dbfs(
            target_dbfs, taps, fc, fs);
        NoiseConfig cfg;
        cfg.wenz_sea_state        = ss;
        cfg.target_level_db_re_fs = rms_dbfs;
        cfg.saltwater             = true;
        NoiseGenerator gen(cfg, static_cast<uint32_t>(fs));
        gen.set_seed(7);
        constexpr size_t segment_len = 8192;
        constexpr size_t n_segments  = 200;
        std::vector<float> out(segment_len * n_segments);
        gen.generate(out.data(), out.size());
        return measured_amp_psd_dbfs(out, fc, fs, segment_len, n_segments);
    };

    const double m_lo = run(-100.0f);
    const double m_hi = run(-80.0f);
    EXPECT_NEAR(m_hi - m_lo, 20.0, 0.5);
    EXPECT_NEAR(m_lo, -100.0, 1.5);
    EXPECT_NEAR(m_hi,  -80.0, 1.5);
}

// ===========================================================================
// Empirical: psd_target_to_total_rms_dbfs hits the target at frequencies far
// from the shaping filter's spectral peak. Catches |H(fc)| handling errors.
// ===========================================================================

TEST(NoiseLevels, OutputPsdMatchesTargetOffShapingPeak) {
    constexpr float fs = 500'000.0f;
    constexpr int   ss = 3;
    constexpr float target_psd_dbfs = -90.0f;
    const auto taps = design_shaping_filter(fs, ss, true, 32);

    // Wenz shape peaks at the low end (wind component); test fc = 60 kHz
    // sits closer to the Wenz minimum, which exercises a small |H(fc)|.
    constexpr float fc = 60'000.0f;
    const float rms_dbfs = psd_target_to_total_rms_dbfs(
        target_psd_dbfs, taps, fc, fs);
    NoiseConfig cfg;
    cfg.wenz_sea_state        = ss;
    cfg.target_level_db_re_fs = rms_dbfs;
    cfg.saltwater             = true;
    NoiseGenerator gen(cfg, static_cast<uint32_t>(fs));
    gen.set_seed(11);

    constexpr size_t segment_len = 8192;
    constexpr size_t n_segments  = 200;
    std::vector<float> out(segment_len * n_segments);
    gen.generate(out.data(), out.size());

    const double measured = measured_amp_psd_dbfs(out, fc, fs, segment_len, n_segments);
    EXPECT_NEAR(measured, target_psd_dbfs, 1.5);
}

// Sweep across the (ss, fc) operating points the production YAMLs use and
// verify the at-fc output PSD matches target. Catches ss-dependent biases
// that the single-point ss=3 / fc=31.5 kHz pin would miss — e.g. if the
// 32-tap Hann-windowed shape happens to under-represent the spectral peak
// at low sea states.
TEST(NoiseLevels, OutputPsdAtFcMatchesTargetAcrossSeaStates) {
    constexpr float fs = 500'000.0f;
    constexpr float fc = 31'500.0f;
    constexpr float target_psd_dbfs = -80.0f;
    constexpr size_t segment_len = 8192;
    constexpr size_t n_segments  = 400;  // tighter SE → ≈0.3 dB

    for (int ss : {0, 1, 2, 3, 4, 5, 6}) {
        const auto taps = design_shaping_filter(fs, ss, true, 32);
        const float rms_dbfs = psd_target_to_total_rms_dbfs(
            target_psd_dbfs, taps, fc, fs);
        NoiseConfig cfg;
        cfg.wenz_sea_state        = ss;
        cfg.target_level_db_re_fs = rms_dbfs;
        cfg.saltwater             = true;
        NoiseGenerator gen(cfg, static_cast<uint32_t>(fs));
        gen.set_seed(20260503u + static_cast<uint64_t>(ss));

        std::vector<float> out(segment_len * n_segments);
        gen.generate(out.data(), out.size());
        const double measured =
            measured_amp_psd_dbfs(out, fc, fs, segment_len, n_segments);

        EXPECT_NEAR(measured, target_psd_dbfs, 1.5)
            << "ss=" << ss << " measured=" << measured
            << " err=" << (measured - target_psd_dbfs);
        std::printf("ss=%d  target=%.2f  measured=%.3f  err=%+.3f dB\n",
                    ss, target_psd_dbfs, measured,
                    measured - target_psd_dbfs);
    }
}
