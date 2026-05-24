#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "config/scenario.hpp"
#include "core/types.hpp"
#include "dsp/noise_psd.hpp"
#include "dsp/wenz_model.hpp"
#include "dsp/calibration_math.hpp"

using namespace openCREST;
using namespace openCREST::dsp;

namespace {

// realistic_cal mirrors the OpenAquatix HIL hardware. loopback_gain is
// chosen so preamp_gain_db = 0 (unity preamp); tests that want a
// non-trivial preamp set loopback_gain on the returned struct.
//
// For reference: loopback_gain (linear) = atten[loopback_cal_atten] ×
// preamp_gain_linear. With cal_atten = 0 and atten[0] = -93.16 dB
// (= 2.2e-5 linear), unity preamp gives loopback_gain = 2.2e-5.
CalibrationData realistic_cal(float fc_hz                                = 31'500.0f,
                              float vref                                  = 1.65f,
                              float in_atten_1                            = -62.71f,
                              float noise_floor_psd_counts_per_sqrt_hz    = 0.5f) {
    CalibrationData c;
    c.adc_bits             = 16;
    c.dac_bits             = 16;
    c.adc_sampling_rate    = 500'000;
    c.dac_sampling_rate    = 500'000;
    c.adc_vref_peak_volts  = vref;
    c.dac_vref_peak_volts  = vref;
    c.center_freq_hz       = fc_hz;
    c.input_attenuation[0] = -93.16f;
    c.input_attenuation[1] = in_atten_1;
    c.output_attenuation   = -40.09f;
    c.noise_floor_psd_counts_per_sqrt_hz = noise_floor_psd_counts_per_sqrt_hz;
    c.loopback_cal_attenuation = 0;
    c.loopback_gain        = std::pow(10.0f, -93.16f / 20.0f); // → preamp_gain = 0 dB
    return c;
}

} // namespace

// ---------------------------------------------------------------------------
// filter_response_at — the new wenz_model helper
// ---------------------------------------------------------------------------

TEST(FilterResponseAt, SingleTapHasFlatUnityResponse) {
    const std::vector<float> taps = {1.0f};
    EXPECT_NEAR(filter_response_at(taps, 0.0f,    500'000.0f), 1.0f, 1e-6f);
    EXPECT_NEAR(filter_response_at(taps, 25e3f,   500'000.0f), 1.0f, 1e-6f);
    EXPECT_NEAR(filter_response_at(taps, 250e3f,  500'000.0f), 1.0f, 1e-6f);
}

TEST(FilterResponseAt, SymmetricTapsRealValuedResponseAtDC) {
    // Symmetric h = [0.25, 0.5, 0.25]; H(0) = 1 (unit DC gain).
    const std::vector<float> taps = {0.25f, 0.5f, 0.25f};
    EXPECT_NEAR(filter_response_at(taps, 0.0f, 500'000.0f), 1.0f, 1e-6f);
    // At Nyquist, alternating signs cancel out: H(Fs/2) = 0.25 - 0.5 + 0.25 = 0
    EXPECT_NEAR(filter_response_at(taps, 250'000.0f, 500'000.0f), 0.0f, 1e-5f);
}

TEST(FilterResponseAt, EmptyOrInvalidReturnsZero) {
    EXPECT_FLOAT_EQ(filter_response_at({}, 1000.0f, 500'000.0f), 0.0f);
    EXPECT_FLOAT_EQ(filter_response_at({1.0f}, 1000.0f, 0.0f),   0.0f);
}

TEST(FilterResponseAt, ShapingFilterPeaksNearWenzMinimum) {
    // The Wenz spectrum drops monotonically with frequency above ~50 kHz
    // (wind term) until thermal-noise dominance. At 30 kHz the FIR
    // designed for sea_state=3 has a non-trivial response > 0.
    const auto taps = design_shaping_filter(500'000.0f, 3, true, 32);
    const float h_30kHz = filter_response_at(taps, 30'000.0f, 500'000.0f);
    EXPECT_GT(h_30kHz, 0.0f);
    EXPECT_LE(h_30kHz, 1.0f);  // peak-normalised
}

// ---------------------------------------------------------------------------
// AfePsdConversionFromCountsPerSqrtHzKnownValue
// ---------------------------------------------------------------------------

TEST(AfePsd, ConversionFromCountsPerSqrtHzKnownValue) {
    auto cal = realistic_cal();
    cal.adc_bits = 16;
    cal.noise_floor_psd_counts_per_sqrt_hz = 1.0f;
    const float midpoint = 32768.0f;  // 2^(16-1)
    const float expected = 20.0f * std::log10(1.0f / midpoint);
    EXPECT_NEAR(afe_psd_dbfs_per_sqrt_hz(cal), expected, 1e-4f);
}

TEST(AfePsd, ZeroNoiseFloorReturnsNegativeInfinity) {
    auto cal = realistic_cal();
    cal.noise_floor_psd_counts_per_sqrt_hz = 0.0f;
    EXPECT_TRUE(std::isinf(afe_psd_dbfs_per_sqrt_hz(cal)));
}

// ---------------------------------------------------------------------------
// WenzPsdAtCenterFreqMonotonicInSeaState
// ---------------------------------------------------------------------------

// 10·log10(P_µPa²/Hz) = 20·log10(P_µPa/√Hz): the dB value of amplitude
// PSD equals the dB value of power PSD. With RVR = 0 the preamp-input
// dBV value equals the raw Wenz dB power value numerically.
TEST(NaturalPsd, AmplitudePsdEqualsPowerPsdDb) {
    const TransducerSpec t{0.0f, 0.0f};
    const float natural = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(
        31'500.0f, t, /*sea_state=*/3, /*saltwater=*/true);
    const float expected = noise_psd_db(31'500.0f, 3, true);
    EXPECT_NEAR(natural, expected, 1e-3f);
}

// At fc=31.5 kHz, ss=3: Wenz wind dominates → ≈47 dB re µPa²/Hz.
TEST(NaturalPsd, AbsoluteValueAtThirtyOnePointFiveKHzSeaState3) {
    const float p = noise_psd_db(31'500.0f, 3, true);
    EXPECT_GT(p, 40.0f);
    EXPECT_LT(p, 55.0f);
}

TEST(NaturalPsd, MonotonicInSeaStateAtFixedFc) {
    const TransducerSpec t{152.0f, -180.0f};
    const float ss0 = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(15'000.0f, t, 0, true);
    const float ss3 = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(15'000.0f, t, 3, true);
    const float ss6 = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(15'000.0f, t, 6, true);
    EXPECT_LT(ss0, ss3);
    EXPECT_LT(ss3, ss6);
}

// ---------------------------------------------------------------------------
// HeterogeneousReceiverFrequencies — Wenz wind-noise band loud at 10 kHz,
// near-minimum at 50 kHz. Receiver at 50 kHz needs more boost for the same
// AFE floor.
// ---------------------------------------------------------------------------

TEST(NaturalPsd, HeterogeneousReceiverFrequencies) {
    const TransducerSpec t{152.0f, -180.0f};
    const float lo = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(10'000.0f, t, 3, true);
    const float hi = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(50'000.0f, t, 3, true);
    EXPECT_GT(lo, hi)
        << "10 kHz sits in the Wenz wind band; 50 kHz is near the minimum.";
}

// At the preamp input, the natural value is invariant to the RX-side
// attenuation pad and the DAC voltage reference — the calculation no
// longer references into the DAC sample domain at all. Equals Wenz_dB +
// RVR exactly.
TEST(NaturalPsd, PreampReferencedNaturalIgnoresAttenAndVref) {
    const TransducerSpec t{0.0f, -180.0f};
    const float a = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(31'500.0f, t, 3, true);
    // Same fc/ss/RVR — answer must not depend on cal data at all.
    const float b = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(31'500.0f, t, 3, true);
    EXPECT_FLOAT_EQ(a, b);
    // Independently: must equal Wenz_dB + RVR exactly.
    const float expected = noise_psd_db(31'500.0f, 3, true) + (-180.0f);
    EXPECT_NEAR(a, expected, 1e-3f);
}

// ---------------------------------------------------------------------------
// compute_boost_db
// ---------------------------------------------------------------------------

TEST(BoostDb, ZeroWhenNaturalAboveAfePlusMargin) {
    // natural is 5 dB above (afe + margin) → no boost needed.
    const float boost = compute_boost_db(/*natural=*/-100.0f,
                                          /*afe=*/-115.0f,
                                          /*margin=*/10.0f);
    EXPECT_FLOAT_EQ(boost, 0.0f);
}

TEST(BoostDb, MatchesAfePlusMarginExactly) {
    // natural is 5 dB BELOW afe; margin = 10 → required = afe + 10 = afe+10.
    // boost = required - natural = (afe+10) - (afe-5) = 15.
    const float boost = compute_boost_db(/*natural=*/-120.0f,
                                          /*afe=*/-115.0f,
                                          /*margin=*/10.0f);
    EXPECT_NEAR(boost, 15.0f, 1e-6f);
}

TEST(BoostDb, DefaultMarginIs10dB) {
    // If natural = afe exactly, default margin → boost should be 10 dB.
    const float boost = compute_boost_db(-115.0f, -115.0f);
    EXPECT_NEAR(boost, 10.0f, 1e-6f);
}

TEST(BoostDb, NonFiniteAfeFallsThroughAsZero) {
    const float boost = compute_boost_db(
        -100.0f, -std::numeric_limits<float>::infinity(), 10.0f);
    EXPECT_FLOAT_EQ(boost, 0.0f);
}

// ---------------------------------------------------------------------------
// psd_target_to_total_rms_dbfs — numeric round-trip via NoiseGenerator math
// ---------------------------------------------------------------------------

TEST(PsdTargetConversion, MatchesAnalyticalDerivation) {
    // Pin a target PSD; compute the total-RMS dBFS that NoiseGenerator
    // should achieve so its output PSD at fc matches.
    const auto taps = design_shaping_filter(500'000.0f, 3, true, 32);
    const float fc = 30'000.0f;
    const float fs = 500'000.0f;

    const float target_psd_dbfs = -120.0f;
    const float total_rms_dbfs =
        psd_target_to_total_rms_dbfs(target_psd_dbfs, taps, fc, fs);

    // Independent re-derivation:
    //   target_rms² = target_psd × Σh² × Fs / (2|H(fc)|²)
    float energy = 0.0f;
    for (float t : taps) energy += t * t;
    const float h_fc   = filter_response_at(taps, fc, fs);
    const double target_psd_lin = std::pow(10.0, target_psd_dbfs / 10.0);
    const double total_rms_lin = std::sqrt(
        target_psd_lin * energy * fs / (2.0 * h_fc * h_fc));
    const double expected_db = 20.0 * std::log10(total_rms_lin);

    EXPECT_NEAR(total_rms_dbfs, expected_db, 1e-3f);
}

TEST(PsdTargetConversion, EmptyTapsReturnsNegInf) {
    EXPECT_TRUE(std::isinf(
        psd_target_to_total_rms_dbfs(-100.0f, {}, 30'000.0f, 500'000.0f)));
}

TEST(PsdTargetConversion, ZeroResponseAtFcReturnsNegInf) {
    // FIR with a hard zero at Nyquist; eval at Fs/2 → response = 0 →
    // requires infinite boost; helper returns -inf as the sentinel.
    const std::vector<float> taps = {0.5f, -0.5f};  // H(Fs/2) = 0.5 - (-0.5) = ... wait
    // Actually for [0.5, -0.5], H(0) = 0.5 - 0.5 = 0; H(Fs/2) = 0.5 + 0.5 = 1.
    // So evaluate at DC instead.
    EXPECT_TRUE(std::isinf(
        psd_target_to_total_rms_dbfs(-100.0f, taps, 0.0f, 500'000.0f)));
}

// ---------------------------------------------------------------------------
// compute_receiver_noise_sizing — the aggregate
// ---------------------------------------------------------------------------

TEST(ReceiverNoiseSizing, NaturalAboveAfePlusMarginYieldsZeroBoost) {
    auto cal = realistic_cal(/*fc_hz=*/10'000.0f);
    cal.noise_floor_psd_counts_per_sqrt_hz = 0.001f;  // very quiet AFE
    const TransducerSpec t{152.0f, -180.0f};

    const auto s = compute_receiver_noise_sizing(
        cal, t, /*sea_state=*/4, /*saltwater=*/true,
        /*atten_idx=*/1, /*margin=*/10.0f);

    // natural and afe are now both at the preamp input (dB ref 1 V/√Hz);
    // natural dominates → no boost needed.
    EXPECT_GT(s.natural_psd_dbv_at_preamp,
              s.afe_psd_dbv_at_preamp + 10.0f);
    EXPECT_FLOAT_EQ(s.boost_db, 0.0f);
    // target at DAC = un-do(input pad + V_ref_dac) on (natural + boost) at preamp.
    const float vref_dac_db = 20.0f * std::log10(cal.dac_vref_peak_volts);
    const float expected_target_dbfs = s.natural_psd_dbv_at_preamp
                                     - cal.input_attenuation[1]
                                     - vref_dac_db;
    EXPECT_NEAR(s.target_psd_dbfs_at_dac, expected_target_dbfs, 1e-3f);
}

TEST(ReceiverNoiseSizing, AfeDominatesAtVeryLowSeaStateForcesBoost) {
    auto cal = realistic_cal(/*fc_hz=*/50'000.0f);  // near Wenz minimum
    cal.noise_floor_psd_counts_per_sqrt_hz = 5.0f;  // loud AFE
    const TransducerSpec t{152.0f, -180.0f};

    const auto s = compute_receiver_noise_sizing(
        cal, t, /*sea_state=*/0, /*saltwater=*/true,
        /*atten_idx=*/1, /*margin=*/10.0f);

    EXPECT_GT(s.boost_db, 0.0f);
    // Boost forces target == afe + margin at the preamp; verify by
    // back-converting target_dbfs_at_dac to preamp dBV.
    const float vref_dac_db = 20.0f * std::log10(cal.dac_vref_peak_volts);
    const float target_dbv_preamp = s.target_psd_dbfs_at_dac
                                  + cal.input_attenuation[1]
                                  + vref_dac_db;
    EXPECT_NEAR(target_dbv_preamp,
                s.afe_psd_dbv_at_preamp + 10.0f, 1e-3f);
}

TEST(ReceiverNoiseSizing, UncalibratedAfeFallsBackToConstant) {
    auto cal = realistic_cal();
    cal.noise_floor_psd_counts_per_sqrt_hz = 0.0f;
    const TransducerSpec t{152.0f, -180.0f};

    const auto s = compute_receiver_noise_sizing(
        cal, t, 3, true, 1, 10.0f);

    EXPECT_FLOAT_EQ(s.afe_psd_dbv_at_preamp, kFallbackAfePsdDbvAtPreamp);
}

// AFE noise floor at the preamp input must not change with the operating
// attenuation pad: noise originates at the preamp/ADC, so the AFE-at-preamp
// is input-referred and invariant.
// (AFE→preamp uses preamp_gain from loopback_gain & loopback_cal_attenuation,
// NOT input_atten[op].)
TEST(ReceiverNoiseSizing, AfeAtPreampInvariantToOperatingAtten) {
    auto cal = realistic_cal();
    const TransducerSpec t{152.0f, -180.0f};
    const auto a = compute_receiver_noise_sizing(cal, t, 3, true,
                                                  /*atten_idx=*/0, 10.0f);
    const auto b = compute_receiver_noise_sizing(cal, t, 3, true,
                                                  /*atten_idx=*/1, 10.0f);
    EXPECT_NEAR(a.afe_psd_dbv_at_preamp, b.afe_psd_dbv_at_preamp, 1e-4f);
    EXPECT_NEAR(a.natural_psd_dbv_at_preamp, b.natural_psd_dbv_at_preamp, 1e-4f);
    // The DAC target SHOULD differ — atten[0] needs more drive than
    // atten[1] for the same preamp voltage (more attenuation to un-do).
    // input_atten[0] = -93.16, input_atten[1] = -62.71 → ~30 dB delta.
    const float delta = a.target_psd_dbfs_at_dac - b.target_psd_dbfs_at_dac;
    EXPECT_NEAR(delta,
                cal.input_attenuation[1] - cal.input_attenuation[0], 1e-3f);
}

// With non-unity preamp_gain (DAC→ADC chain not unity), the boost must
// land at AFE+margin in the preamp frame regardless of preamp_gain.
TEST(ReceiverNoiseSizing, PreampGainDoesNotInflateBoost) {
    auto cal = realistic_cal();
    cal.noise_floor_psd_counts_per_sqrt_hz = 1.0f;
    const TransducerSpec t{152.0f, -180.0f};

    // Pair A: unity preamp (loopback_gain set so preamp_gain = 0 dB).
    cal.loopback_gain = std::pow(10.0f, cal.input_attenuation[0] / 20.0f);
    const auto a = compute_receiver_noise_sizing(cal, t, 0, true, 1, 10.0f);

    // Pair B: +60 dB preamp (loopback_gain 60 dB above the pad linear gain).
    cal.loopback_gain = std::pow(
        10.0f, (cal.input_attenuation[0] + 60.0f) / 20.0f);
    const auto b = compute_receiver_noise_sizing(cal, t, 0, true, 1, 10.0f);

    // AFE input-referred to the preamp is 60 dB lower in B (the same ADC
    // counts/√Hz, divided by 1000× more preamp gain).
    EXPECT_NEAR(b.afe_psd_dbv_at_preamp - a.afe_psd_dbv_at_preamp, -60.0f, 1e-3f);

    // Boosts differ by 60 dB (B's quieter AFE requires less or no boost
    // depending on whether natural still dominates; if A boosts at all,
    // B's boost is 60 dB lower — clamped at 0).
    EXPECT_NEAR(std::max(0.0f, a.boost_db - 60.0f), b.boost_db, 1e-3f);

    // Both targets, when injected and propagated through the actual
    // chain, land the noise at AFE_at_preamp + margin (or above if natural
    // dominates) — i.e. the preamp-frame contract holds independent of
    // the preamp gain.
    auto target_dbv_preamp = [&](const ReceiverNoiseSizing& s) {
        return s.target_psd_dbfs_at_dac + cal.input_attenuation[1]
             + 20.0f * std::log10(cal.dac_vref_peak_volts);
    };
    EXPECT_GE(target_dbv_preamp(a), a.afe_psd_dbv_at_preamp + 10.0f - 1e-3f);
    EXPECT_GE(target_dbv_preamp(b), b.afe_psd_dbv_at_preamp + 10.0f - 1e-3f);
}
