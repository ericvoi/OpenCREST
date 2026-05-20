#include <gtest/gtest.h>
#include <cmath>

#include "config/scenario.hpp"
#include "core/types.hpp"
#include "dsp/path_loss.hpp"
#include "dsp/physical_gain.hpp"

using openCREST::CalibrationData;
using openCREST::ChannelConfig;
using openCREST::TransducerSpec;
using openCREST::dsp::compute_channel_physical_gain_db;

namespace {

// Construct a CalibrationData with the new Phase A voltage / fc fields
// explicitly set (so tests pin the contract regardless of struct defaults).
CalibrationData cal(float vref_adc_peak, float vref_dac_peak,
                    float fc_hz,
                    float output_atten_db = 0.0f,
                    float input_atten0_db = 0.0f,
                    float input_atten1_db = 0.0f) {
    CalibrationData c;
    c.adc_bits             = 16;
    c.dac_bits             = 16;
    c.adc_sampling_rate    = 500'000;
    c.dac_sampling_rate    = 500'000;
    c.adc_vref_peak_volts  = vref_adc_peak;
    c.dac_vref_peak_volts  = vref_dac_peak;
    c.center_freq_hz       = fc_hz;
    c.output_attenuation   = output_atten_db;
    c.input_attenuation[0] = input_atten0_db;
    c.input_attenuation[1] = input_atten1_db;
    return c;
}

ChannelConfig minimal_cfg(float range_m,
                          float spreading_factor = 2.0f,
                          bool  saltwater        = true) {
    ChannelConfig cc;
    cc.from_modem       = "A";
    cc.to_modem         = "B";
    cc.range_m          = range_m;
    cc.spreading_factor = spreading_factor;
    cc.saltwater        = saltwater;
    cc.sound_speed_m_s  = 1500.0f;
    return cc;
}

} // namespace

// ---------------------------------------------------------------------------
// KnownChainEndToEnd: pinned numeric example from the plan, computed against
// the exact dB-sum the formula promises. Tight tolerance (1 mdB).
// ---------------------------------------------------------------------------
TEST(PhysicalGain, KnownChainEndToEnd) {
    constexpr float fc_hz    = 25'000.0f;
    constexpr float vref     = 1.65f;
    constexpr float tvr_db   = 152.0f;
    constexpr float rvr_db   = -180.0f;
    constexpr float out_at   = -60.0f;
    constexpr float in_at0   = -10.0f;
    constexpr float range_m  = 100.0f;

    const auto src_cal = cal(vref, vref, fc_hz, out_at, in_at0, 0.0f);
    const auto rx_cal  = src_cal;
    const TransducerSpec src_tx{tvr_db, rvr_db};
    const TransducerSpec rx_tx = src_tx;

    auto cfg = minimal_cfg(range_m);
    cfg.rx_atten_idx = 0;

    const double got = compute_channel_physical_gain_db(
        src_cal, rx_cal, src_tx, rx_tx, cfg, /*atten_idx=*/0);

    // Independent reference: the very-explicit dB sum, computed in double.
    const double tl_db = openCREST::dsp::transmission_loss_db(
        range_m, fc_hz / 1000.0f, cfg.spreading_factor, cfg.saltwater);
    const double v_db  = 20.0 * std::log10(static_cast<double>(vref));
    const double expected = v_db + (-out_at) + tvr_db
                          - tl_db + rvr_db + (-in_at0) - v_db;

    EXPECT_NEAR(got, expected, 1e-3) << "tl_db=" << tl_db;
    // And: the value lives in a sane neighbourhood for these knobs.
    EXPECT_GT(got, -5.0);
    EXPECT_LT(got,  5.0);
}

// ---------------------------------------------------------------------------
// DoublingRangeAdds6dBSpherical: with all other knobs fixed, doubling the
// range under spherical spreading must drop the chain by exactly the
// spherical-spreading delta (≈ 6.02 dB) plus a small Thorp absorption
// increment from the extra range. The relationship is path-loss only —
// every other dB contribution cancels out.
// ---------------------------------------------------------------------------
TEST(PhysicalGain, DoublingRangeAdds6dBSpherical) {
    constexpr float fc_hz = 25'000.0f;
    const auto c = cal(1.0f, 1.0f, fc_hz);
    const TransducerSpec tx{0.0f, 0.0f};

    auto cfg_short = minimal_cfg(50.0f);
    auto cfg_long  = minimal_cfg(100.0f);

    const double g_short = compute_channel_physical_gain_db(c, c, tx, tx, cfg_short, 0);
    const double g_long  = compute_channel_physical_gain_db(c, c, tx, tx, cfg_long,  0);

    const double tl_short = openCREST::dsp::transmission_loss_db(
        50.0f,  fc_hz / 1000.0f, 2.0f, true);
    const double tl_long  = openCREST::dsp::transmission_loss_db(
        100.0f, fc_hz / 1000.0f, 2.0f, true);

    EXPECT_NEAR(g_short - g_long, tl_long - tl_short, 1e-3);
    // Spherical spreading alone contributes 6.0206 dB to that delta.
    EXPECT_GT(g_short - g_long, 6.0);
    EXPECT_LT(g_short - g_long, 7.0);
}

// ---------------------------------------------------------------------------
// TvrRvrSignSymmetry: shifting tvr by +x and rvr by -x must leave the chain
// unchanged (TVR and RVR enter the dB sum with equal weight, opposite
// roles).
// ---------------------------------------------------------------------------
TEST(PhysicalGain, TvrRvrSignSymmetry) {
    const auto c = cal(2.5f, 2.5f, 30'000.0f, -40.0f, -5.0f, 0.0f);
    auto cfg = minimal_cfg(75.0f);

    const TransducerSpec tx_a{ 152.0f, -180.0f };
    const TransducerSpec tx_b{ 162.0f, -190.0f };  // +10 dB tvr, -10 dB rvr

    const double g_a = compute_channel_physical_gain_db(c, c, tx_a, tx_a, cfg, 0);
    const double g_b = compute_channel_physical_gain_db(c, c, tx_b, tx_b, cfg, 0);

    EXPECT_NEAR(g_a, g_b, 1e-4);
}

// ---------------------------------------------------------------------------
// LegacyDefaultsCollapseToPathLossOnly: with default CalibrationData{}
// (Vref=1.0, atten=0, fc=25 kHz) and TransducerSpec{} (tvr=rvr=0), the chain
// must reduce to exactly -TL_dB at the source's center frequency. This is
// the load-bearing invariant for shipping Phase B before firmware updates.
// ---------------------------------------------------------------------------
TEST(PhysicalGain, LegacyDefaultsCollapseToPathLossOnly) {
    const CalibrationData default_cal{};
    const TransducerSpec  default_tx{};
    auto cfg = minimal_cfg(100.0f);

    const double got = compute_channel_physical_gain_db(
        default_cal, default_cal, default_tx, default_tx, cfg, 0);

    const double expected_tl = openCREST::dsp::transmission_loss_db(
        cfg.range_m, default_cal.center_freq_hz / 1000.0f,
        cfg.spreading_factor, cfg.saltwater);

    EXPECT_NEAR(got, -expected_tl, 1e-4);
}

// ---------------------------------------------------------------------------
// HeterogeneousCenterFrequencies: path loss must be evaluated at the SOURCE
// modem's center frequency, not the receiver's. Set src_cal.fc = 10 kHz and
// rx_cal.fc = 50 kHz with all other knobs equal; the chain must match a
// reference TL computed at 10 kHz.
// ---------------------------------------------------------------------------
TEST(PhysicalGain, HeterogeneousCenterFrequencies) {
    const auto src_cal = cal(1.0f, 1.0f, 10'000.0f);
    const auto rx_cal  = cal(1.0f, 1.0f, 50'000.0f);
    const TransducerSpec tx{0.0f, 0.0f};
    auto cfg = minimal_cfg(200.0f);

    const double got = compute_channel_physical_gain_db(
        src_cal, rx_cal, tx, tx, cfg, 0);

    const double expected_tl_at_src =
        openCREST::dsp::transmission_loss_db(
            cfg.range_m, src_cal.center_freq_hz / 1000.0f,
            cfg.spreading_factor, cfg.saltwater);

    EXPECT_NEAR(got, -expected_tl_at_src, 1e-4);

    // Sanity: the wrong-frequency answer must NOT match (50 kHz Thorp >>
    // 10 kHz Thorp at 200 m).
    const double expected_tl_at_rx =
        openCREST::dsp::transmission_loss_db(
            cfg.range_m, rx_cal.center_freq_hz / 1000.0f,
            cfg.spreading_factor, cfg.saltwater);
    EXPECT_GT(std::abs(expected_tl_at_rx - expected_tl_at_src), 1.0);
}

// ---------------------------------------------------------------------------
// AttenIdxSelectsRxInputAttenuation: the chain must read
// rx_cal.input_attenuation[atten_idx], not [0] always.
// ---------------------------------------------------------------------------
TEST(PhysicalGain, AttenIdxSelectsRxInputAttenuation) {
    auto rx = cal(1.0f, 1.0f, 25'000.0f, /*out_at=*/0.0f,
                  /*in0=*/-10.0f, /*in1=*/-30.0f);
    const auto src = cal(1.0f, 1.0f, 25'000.0f);
    const TransducerSpec tx{0.0f, 0.0f};
    auto cfg = minimal_cfg(50.0f);

    const double g_idx0 = compute_channel_physical_gain_db(src, rx, tx, tx, cfg, 0);
    const double g_idx1 = compute_channel_physical_gain_db(src, rx, tx, tx, cfg, 1);

    // idx 1 has 20 dB more attenuation than idx 0 → un-doing it adds 20 dB
    // more gain to the chain.
    EXPECT_NEAR(g_idx1 - g_idx0, 20.0, 1e-4);
}

// ---------------------------------------------------------------------------
// VrefRatioContributesAdcMinusDac: shifting only adc_vref shifts the chain
// by +20·log10(ratio). Shifting only dac_vref shifts it by -20·log10(ratio).
// ---------------------------------------------------------------------------
TEST(PhysicalGain, VrefRatioContributesAdcMinusDac) {
    const auto src_a = cal(1.0f, 2.0f, 25'000.0f);
    const auto src_b = cal(2.0f, 2.0f, 25'000.0f);   // ADC vref ×2 → +6 dB
    const auto rx    = cal(1.0f, 4.0f, 25'000.0f);   // DAC vref ×4 vs A's 2 → -6 dB
    const TransducerSpec tx{0.0f, 0.0f};
    auto cfg = minimal_cfg(20.0f);

    const double g_a = compute_channel_physical_gain_db(src_a, src_a, tx, tx, cfg, 0);
    const double g_b = compute_channel_physical_gain_db(src_b, src_a, tx, tx, cfg, 0);
    const double g_c = compute_channel_physical_gain_db(src_a, rx,    tx, tx, cfg, 0);

    EXPECT_NEAR(g_b - g_a,  20.0 * std::log10(2.0), 1e-4);
    EXPECT_NEAR(g_c - g_a, -20.0 * std::log10(2.0), 1e-4);
}
