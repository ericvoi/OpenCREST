#include <gtest/gtest.h>
#include <cmath>
#include "core/types.hpp"
#include "dsp/calibration_math.hpp"

using namespace openCREST;
using namespace openCREST::dsp;

namespace {

CalibrationData make_realistic_cal() {
    CalibrationData c;
    c.adc_bits                              = 16;
    c.dac_bits                              = 12;
    c.adc_vref_peak_volts                   = 1.65f;
    c.dac_vref_peak_volts                   = 1.65f;
    c.center_freq_hz                        = 25'000.0f;
    c.adc_sampling_rate                     = 500'000;
    c.dac_sampling_rate                     = 500'000;
    c.input_attenuation[0]                  = -10.0f;
    c.input_attenuation[1]                  = -40.0f;
    c.output_attenuation                    = -60.0f;
    c.noise_floor_psd_counts_per_sqrt_hz    = 1.0f;
    return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// adc_dbfs_to_volts_rms / dac_volts_rms_to_dbfs (inverses on identical Vref)
// ---------------------------------------------------------------------------

TEST(CalibrationMath, AdcDbfsToVoltsRmsKnownValue) {
    auto c = make_realistic_cal();
    // 0 dBFS (s_rms = 1.0) → V_rms = V_ref_peak
    EXPECT_NEAR(adc_dbfs_to_volts_rms(0.0f, c), 1.65f, 1e-5f);
    // -20 dBFS (s_rms = 0.1) → V_rms = 0.165 V
    EXPECT_NEAR(adc_dbfs_to_volts_rms(-20.0f, c), 0.165f, 1e-5f);
}

TEST(CalibrationMath, DacVoltsRmsToDbfsKnownValue) {
    auto c = make_realistic_cal();
    EXPECT_NEAR(dac_volts_rms_to_dbfs(1.65f, c), 0.0f, 1e-5f);
    EXPECT_NEAR(dac_volts_rms_to_dbfs(0.165f, c), -20.0f, 1e-4f);
}

TEST(CalibrationMath, AdcDacRoundTripWithSameVref) {
    auto c = make_realistic_cal();
    for (float dbfs : {-60.0f, -20.0f, -3.0f, 0.0f}) {
        const float v = adc_dbfs_to_volts_rms(dbfs, c);
        const float back = dac_volts_rms_to_dbfs(v, c);
        EXPECT_NEAR(back, dbfs, 1e-4f) << "round trip failed at " << dbfs;
    }
}

TEST(CalibrationMath, DacVoltsRmsToDbfsRejectsZeroOrNegativeWithMinusInf) {
    auto c = make_realistic_cal();
    EXPECT_TRUE(std::isinf(dac_volts_rms_to_dbfs(0.0f, c)));
    EXPECT_TRUE(std::isinf(dac_volts_rms_to_dbfs(-1.0f, c)));
    EXPECT_LT(dac_volts_rms_to_dbfs(0.0f, c), 0.0f);  // -inf, not +inf
}

// ---------------------------------------------------------------------------
// volts_rms_at_adc_to_drive_volts_rms — un-do output_attenuation pad
// ---------------------------------------------------------------------------

TEST(CalibrationMath, UndoOutputAttenuationKnownValue) {
    auto c = make_realistic_cal();
    // -60 dB pad means a factor 1000 attenuation; multiply observed ADC
    // voltage by 1000 to recover the drive.
    EXPECT_NEAR(volts_rms_at_adc_to_drive_volts_rms(0.001f, c), 1.0f, 1e-5f);
}

TEST(CalibrationMath, UndoOutputAttenuationZeroIsIdentity) {
    auto c = make_realistic_cal();
    c.output_attenuation = 0.0f;
    EXPECT_FLOAT_EQ(volts_rms_at_adc_to_drive_volts_rms(0.5f, c), 0.5f);
}

// ---------------------------------------------------------------------------
// preamp_volts_rms_to_dac_volts_rms — pre-multiply by inverse of input pad
// ---------------------------------------------------------------------------

TEST(CalibrationMath, ApplyInputAttenuationKnownValue) {
    auto c = make_realistic_cal();
    // input_attenuation[0] = -10 dB → factor 0.3162; we need to drive
    // the DAC at 1.0 / 0.3162 = 3.162 V to land 1.0 V at preamp.
    EXPECT_NEAR(preamp_volts_rms_to_dac_volts_rms(1.0f, c, 0), 3.16228f, 1e-4f);
    // index 1: -40 dB → factor 0.01; need 100 V for 1 V at preamp.
    EXPECT_NEAR(preamp_volts_rms_to_dac_volts_rms(1.0f, c, 1), 100.0f, 1e-3f);
}

TEST(CalibrationMath, ApplyInputAttenuationOutOfRangeIdxIsIdentity) {
    auto c = make_realistic_cal();
    EXPECT_FLOAT_EQ(preamp_volts_rms_to_dac_volts_rms(2.0f, c, 5), 2.0f);
}

// ---------------------------------------------------------------------------
// rx_chain_db_uPa_to_dBFS — single-scalar receiver chain
// ---------------------------------------------------------------------------

TEST(CalibrationMath, RxChainDbKnownValue) {
    auto c = make_realistic_cal();
    // RVR = -180 dB re 1 V/µPa, input_atten[0] = -10, vref_dac = 1.65 V
    //   rx_chain_db = -180 + 10 - 20·log10(1.65)
    //               = -170 - 4.350
    //               ≈ -174.350 dB
    const float rx_chain_db = rx_chain_db_uPa_to_dBFS(c, -180.0f, 0);
    EXPECT_NEAR(rx_chain_db, -174.350f, 0.01f);
}

TEST(CalibrationMath, RxChainAtUnityVrefAndZeroAtten) {
    CalibrationData c;  // defaults: vref=1.0, atten=0
    EXPECT_NEAR(rx_chain_db_uPa_to_dBFS(c, -180.0f, 0), -180.0f, 1e-5f);
}

TEST(CalibrationMath, RxChainSummingProducesCorrectDbfs) {
    auto c = make_realistic_cal();
    const float rx_chain_db = rx_chain_db_uPa_to_dBFS(c, -180.0f, 0);
    // 100 dB re 1 µPa input → 100 + rx_chain_db at the ADC
    const float dbfs = 100.0f + rx_chain_db;
    EXPECT_NEAR(dbfs, -74.350f, 0.01f);
}

// ---------------------------------------------------------------------------
// afe_psd_dbfs_per_sqrt_hz — modem-reported PSD → dBFS/√Hz
// ---------------------------------------------------------------------------

TEST(CalibrationMath, AfePsdKnownValue) {
    auto c = make_realistic_cal();
    // adc_bits=16 → midpoint=32768; psd=1.0 counts/√Hz
    // → dBFS/√Hz = 20·log10(1/32768) ≈ -90.309 dB
    EXPECT_NEAR(afe_psd_dbfs_per_sqrt_hz(c), -90.309f, 0.01f);
}

TEST(CalibrationMath, AfePsdZeroPsdIsMinusInf) {
    auto c = make_realistic_cal();
    c.noise_floor_psd_counts_per_sqrt_hz = 0.0f;
    EXPECT_TRUE(std::isinf(afe_psd_dbfs_per_sqrt_hz(c)));
    EXPECT_LT(afe_psd_dbfs_per_sqrt_hz(c), 0.0f);
}

TEST(CalibrationMath, AfePsdScalesWithBitDepth) {
    // Doubling adc_bits → midpoint scales by 2^(extra bits) → dBFS drops by 6
    // per bit (signal-to-FS ratio shrinks).
    CalibrationData c12;
    c12.adc_bits = 12;
    c12.noise_floor_psd_counts_per_sqrt_hz = 1.0f;

    CalibrationData c16 = c12;
    c16.adc_bits = 16;

    const float diff = afe_psd_dbfs_per_sqrt_hz(c12) - afe_psd_dbfs_per_sqrt_hz(c16);
    // 4 extra bits → midpoint scales by 2^4 → 20·log10(16) ≈ 24.08 dB
    EXPECT_NEAR(diff, 20.0f * std::log10(16.0f), 1e-3f);
}

// ---------------------------------------------------------------------------
// preamp_gain_db — derives intrinsic preamp gain from the modem-reported
// loopback_gain and the calibration attenuation.
// ---------------------------------------------------------------------------

TEST(CalibrationMath, PreampGainDbDefaultCalIsZero) {
    // Default-constructed cal: loopback_gain = 1.0, atten = 0 dB.
    // → preamp_gain_db = 20·log10(1) − 0 = 0 dB (identity preamp).
    CalibrationData c;
    EXPECT_NEAR(preamp_gain_db(c), 0.0f, 1e-5f);
}

TEST(CalibrationMath, PreampGainDbKnownValue) {
    auto c = make_realistic_cal();
    c.loopback_cal_attenuation = 0;        // cal pad at index 0 → -10 dB
    c.loopback_gain            = 0.1f;     // chain at cal: -20 dB
    // preamp_gain_db = 20·log10(0.1) − (-10) = -20 + 10 = -10 dB
    EXPECT_NEAR(preamp_gain_db(c), -10.0f, 1e-3f);
}

TEST(CalibrationMath, PreampGainDbRecoversChainAtCalAttenuation) {
    // Round-trip: chain_at_cal = atten[cal] + preamp_gain (in dB).
    auto c = make_realistic_cal();
    c.loopback_cal_attenuation = 0;
    c.loopback_gain            = 0.05f;    // 20·log10(0.05) ≈ -26.02 dB
    const float expected_chain_db = 20.0f * std::log10(0.05f);
    const float chain_db = preamp_gain_db(c) + c.input_attenuation[0];
    EXPECT_NEAR(chain_db, expected_chain_db, 1e-3f);
}

TEST(CalibrationMath, PreampGainDbZeroLoopbackIsNan) {
    auto c = make_realistic_cal();
    c.loopback_gain = 0.0f;
    EXPECT_TRUE(std::isnan(preamp_gain_db(c)));
}

// ---------------------------------------------------------------------------
// afe_psd_dbv_at_preamp_per_sqrt_hz — input-referred AFE noise floor.
// ---------------------------------------------------------------------------

TEST(CalibrationMath, AfePsdAtPreampUnityPreampMatchesAdcInDbv) {
    // With preamp_gain_db = 0, the AFE at the preamp equals the AFE at the
    // ADC (in dB(V), adjusting for V_ref_adc).
    auto c = make_realistic_cal();
    c.loopback_cal_attenuation = 0;
    c.loopback_gain            = std::pow(10.0f, c.input_attenuation[0] / 20.0f);
    ASSERT_NEAR(preamp_gain_db(c), 0.0f, 1e-3f);

    const float at_adc_dbfs = afe_psd_dbfs_per_sqrt_hz(c);
    const float at_adc_dbv  = at_adc_dbfs + 20.0f * std::log10(c.adc_vref_peak_volts);
    const float at_preamp   = afe_psd_dbv_at_preamp_per_sqrt_hz(c);
    EXPECT_NEAR(at_preamp, at_adc_dbv, 1e-3f);
}

TEST(CalibrationMath, AfePsdAtPreampDropsByPreampGain) {
    // Increasing the preamp gain means the SAME ADC counts/√Hz come from
    // a smaller input voltage at the preamp — i.e. AFE-at-preamp drops by
    // the same dB amount.
    auto c = make_realistic_cal();
    c.loopback_cal_attenuation = 0;
    c.loopback_gain            = std::pow(10.0f, c.input_attenuation[0] / 20.0f);
    const float a = afe_psd_dbv_at_preamp_per_sqrt_hz(c);

    // Bump preamp gain by 40 dB (loopback_gain × 100).
    c.loopback_gain *= 100.0f;
    const float b = afe_psd_dbv_at_preamp_per_sqrt_hz(c);

    EXPECT_NEAR(b - a, -40.0f, 1e-3f);
}

TEST(CalibrationMath, AfePsdAtPreampUncalibratedIsMinusInf) {
    auto c = make_realistic_cal();
    c.noise_floor_psd_counts_per_sqrt_hz = 0.0f;
    EXPECT_TRUE(std::isinf(afe_psd_dbv_at_preamp_per_sqrt_hz(c)));
}

TEST(CalibrationMath, AfePsdAtPreampZeroLoopbackIsNan) {
    auto c = make_realistic_cal();
    c.loopback_gain = 0.0f;
    EXPECT_TRUE(std::isnan(afe_psd_dbv_at_preamp_per_sqrt_hz(c)));
}

// ---------------------------------------------------------------------------
// dac_dbfs_from_preamp_dbv_per_sqrt_hz — preamp dBV → DAC sample dBFS.
// ---------------------------------------------------------------------------

TEST(CalibrationMath, DacFromPreampUnityIsIdentity) {
    // With V_ref_dac = 1.0 and atten = 0, DAC dBFS == preamp dBV.
    CalibrationData c;
    c.dac_vref_peak_volts  = 1.0f;
    c.input_attenuation[0] = 0.0f;
    c.input_attenuation[1] = 0.0f;
    EXPECT_NEAR(dac_dbfs_from_preamp_dbv_per_sqrt_hz(-100.0f, c, 1),
                -100.0f, 1e-5f);
}

TEST(CalibrationMath, DacFromPreampUnDosInputPad) {
    auto c = make_realistic_cal();
    c.dac_vref_peak_volts  = 1.0f;       // isolate the pad-undo factor
    // atten[1] = -40 dB → un-do adds +40 dB to drive the DAC harder
    EXPECT_NEAR(dac_dbfs_from_preamp_dbv_per_sqrt_hz(-100.0f, c, 1),
                -100.0f - c.input_attenuation[1], 1e-3f);
}

TEST(CalibrationMath, DacFromPreampSubtractsVrefDb) {
    auto c = make_realistic_cal();
    c.input_attenuation[0] = 0.0f;
    c.input_attenuation[1] = 0.0f;
    // V_ref_dac = 1.65 → -20·log10(1.65) ≈ -4.35 dB
    EXPECT_NEAR(dac_dbfs_from_preamp_dbv_per_sqrt_hz(0.0f, c, 1),
                -20.0f * std::log10(c.dac_vref_peak_volts), 1e-3f);
}

TEST(CalibrationMath, DacFromPreampOutOfRangeAttenIdxIsZeroPad) {
    auto c = make_realistic_cal();
    c.dac_vref_peak_volts = 1.0f;
    EXPECT_NEAR(dac_dbfs_from_preamp_dbv_per_sqrt_hz(-50.0f, c, 5),
                -50.0f, 1e-5f);
}

// Round-trip: preamp_dbv → dac_dbfs → physical voltage at DAC →
// physical voltage at preamp → preamp_dbv. Verifies inverse symmetry
// of the input-pad un-do and V_ref_dac normalisation.
TEST(CalibrationMath, PreampDacRoundTrip) {
    auto c = make_realistic_cal();
    for (float preamp_dbv : {-200.0f, -160.0f, -120.0f, -80.0f}) {
        const float dac_dbfs =
            dac_dbfs_from_preamp_dbv_per_sqrt_hz(preamp_dbv, c, 1);
        // Reverse: dac sample → V at DAC pin → V at preamp via the pad.
        const float v_at_dac    = std::pow(10.0f, dac_dbfs / 20.0f)
                                   * c.dac_vref_peak_volts;
        const float atten_lin_1 = std::pow(10.0f, c.input_attenuation[1] / 20.0f);
        const float v_at_preamp = v_at_dac * atten_lin_1;
        const float back        = 20.0f * std::log10(v_at_preamp);
        EXPECT_NEAR(back, preamp_dbv, 1e-3f) << "round-trip failed at " << preamp_dbv;
    }
}
