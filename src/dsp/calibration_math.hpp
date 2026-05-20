#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include "core/types.hpp"

namespace openCREST::dsp {

// Conversions between sample-domain (normalized float [-1, +1] / dBFS),
// voltage-domain (V_rms at modem pins), and acoustic-domain (dB re 1 µPa).
//
// Convention: a normalized sample value of 1.0 corresponds to V_ref_peak
// volts above mid-rail at the ADC/DAC pin (single-ended, peak full-scale).
// Therefore for an RMS-sample value s_rms:
//   V_rms_at_pin = s_rms × V_ref_peak
// And dBFS is defined as 20·log10(s_rms), consistent with how
// NoiseGenerator::target_level_db_re_fs is interpreted elsewhere — this
// puts a full-scale sinusoid (peak = 1) at -3.01 dBFS, not 0 dBFS.
//
// `output_attenuation` and `input_attenuation[]` are reported by the
// firmware as dB *gains* (i.e. negative numbers for resistive pads). To
// recover the upstream voltage we apply 10^(-atten_db/20) — for atten_db
// = -60 dB this multiplies by 1000, undoing the pad.
//
// All functions are pure, header-only, and allocation-free; safe to call
// at scenario load and in unit tests, but not on the real-time hot path
// (use precomputed scalars there).

// V_rms at the ADC input pin corresponding to a sample-domain dBFS value.
[[nodiscard]] inline float adc_dbfs_to_volts_rms(float dbfs,
                                                  const CalibrationData& cal) {
    const float s_rms = std::pow(10.0f, dbfs / 20.0f);
    return s_rms * cal.adc_vref_peak_volts;
}

// Sample-domain dBFS corresponding to a V_rms at the DAC output pin.
[[nodiscard]] inline float dac_volts_rms_to_dbfs(float v_rms,
                                                  const CalibrationData& cal) {
    if (cal.dac_vref_peak_volts <= 0.0f || v_rms <= 0.0f) {
        return -std::numeric_limits<float>::infinity();
    }
    return 20.0f * std::log10(v_rms / cal.dac_vref_peak_volts);
}

// Recover the actual transducer drive voltage from the V_rms observed at
// the modem's monitor ADC pin. The modem's `output_attenuation` is the
// gain (in dB, negative) of the resistive pad coupling the drive line to
// the monitor ADC; we un-do it.
[[nodiscard]] inline float volts_rms_at_adc_to_drive_volts_rms(
        float v_rms_adc, const CalibrationData& cal) {
    return v_rms_adc * std::pow(10.0f, -cal.output_attenuation / 20.0f);
}

// Convert V_rms at the receive preamp input to V_rms at the DAC pin where
// the simulator injects. `input_attenuation[atten_idx]` is the gain (in
// dB, negative) of the resistive pad between the DAC and the preamp; we
// pre-multiply so the desired preamp voltage is achieved after the pad.
[[nodiscard]] inline float preamp_volts_rms_to_dac_volts_rms(
        float v_rms_preamp, const CalibrationData& cal, uint8_t atten_idx) {
    const float atten_db = (atten_idx < 2)
        ? cal.input_attenuation[atten_idx]
        : 0.0f;
    return v_rms_preamp * std::pow(10.0f, -atten_db / 20.0f);
}

// End-to-end RX-chain gain from acoustic pressure (dB re 1 µPa) to ADC
// dBFS, parameterized only by the receiver-side knobs:
//
//   dBFS_at_adc = dB_re_1µPa + rx_chain_db
//
// Derivation:
//   V_preamp = µPa × 10^(rvr_db/20)
//   V_dac    = V_preamp × 10^(-input_atten/20)
//   s_rms    = V_dac / V_ref_dac_peak
//   dBFS     = 20·log10(s_rms)
//            = (dB_re_1µPa + rvr_db) + (-input_atten_db) - 20·log10(V_ref_dac_peak)
//
// `rvr_db` is taken from the YAML-configured TransducerSpec — it does not
// live in CalibrationData (firmware reports voltage/electrical knobs only).
[[nodiscard]] inline float rx_chain_db_uPa_to_dBFS(const CalibrationData& cal,
                                                    float rvr_db,
                                                    uint8_t atten_idx) {
    const float atten_db = (atten_idx < 2)
        ? cal.input_attenuation[atten_idx]
        : 0.0f;
    const float vref = (cal.dac_vref_peak_volts > 0.0f)
        ? cal.dac_vref_peak_volts
        : 1.0f;
    return rvr_db + (-atten_db) - 20.0f * std::log10(vref);
}

// Convert the modem-reported AFE noise floor (PSD in ADC counts / √Hz) to
// dBFS-amplitude per √Hz at the ADC pin. Returns -inf if PSD is zero
// (uncalibrated). The 2^(adc_bits-1) divisor matches the adc_to_float
// full-scale convention.
//
// Note: this is the noise floor as observed at the ADC sample, which sits
// downstream of the preamp. To compare against simulator-side quantities
// referenced at the preamp input, use afe_psd_dbv_at_preamp_per_sqrt_hz.
[[nodiscard]] inline float afe_psd_dbfs_per_sqrt_hz(const CalibrationData& cal) {
    if (cal.noise_floor_psd_counts_per_sqrt_hz <= 0.0f) {
        return -std::numeric_limits<float>::infinity();
    }
    const uint8_t bits = (cal.adc_bits > 0) ? cal.adc_bits : 16;
    const float midpoint = static_cast<float>(1u << (bits - 1));
    return 20.0f * std::log10(cal.noise_floor_psd_counts_per_sqrt_hz / midpoint);
}

// Pre-amplifier voltage gain (dB), derived from the modem-reported
// loopback_gain and the calibration attenuation index.
//
// The firmware measures loopback_gain as the linear DAC-sample → ADC-sample
// gain at the calibration attenuation pad. Decomposing:
//   loopback_gain (linear) = atten[loopback_cal_atten] × preamp_gain
// so:
//   preamp_gain_dB = 20·log10(loopback_gain) − input_atten[loopback_cal_atten]
//
// (input_atten is in dB, negative for a resistive divider pad; subtracting
// a negative adds its magnitude.)
//
// Returns NaN if loopback_gain is non-positive (uncalibrated). With a
// default-constructed CalibrationData (loopback_gain = 1.0, attenuations =
// 0 dB, cal_atten = 0) this evaluates to 0 dB — i.e. an identity preamp,
// which is the legacy assumption baked into older callers.
[[nodiscard]] inline float preamp_gain_db(const CalibrationData& cal) {
    if (cal.loopback_gain <= 0.0f) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const uint16_t cal_idx = (cal.loopback_cal_attenuation < 2)
        ? cal.loopback_cal_attenuation : uint16_t{0};
    const float cal_atten_db = cal.input_attenuation[cal_idx];
    return 20.0f * std::log10(cal.loopback_gain) - cal_atten_db;
}

// AFE noise floor input-referred to the preamp input, in dB ref 1 V/√Hz
// (one-sided amplitude).
//
// Chain: counts/√Hz at ADC → V/√Hz at ADC pin → V/√Hz at preamp input.
//   V_at_ADC = (counts/midpoint) × V_ref_adc_peak
//   V_at_preamp = V_at_ADC / preamp_gain_linear
// In dB:
//   afe_dBV_at_preamp = afe_dBFS_at_adc + 20·log10(V_ref_adc_peak)
//                     − preamp_gain_dB
//
// Returns -inf if AFE PSD is uncalibrated; returns NaN if loopback_gain is
// non-positive (preamp gain unknowable). Both sentinels propagate to
// compute_boost_db, which falls through as zero boost.
[[nodiscard]] inline float afe_psd_dbv_at_preamp_per_sqrt_hz(const CalibrationData& cal) {
    const float afe_dbfs = afe_psd_dbfs_per_sqrt_hz(cal);
    if (!std::isfinite(afe_dbfs)) return afe_dbfs;
    const float pg_db = preamp_gain_db(cal);
    if (!std::isfinite(pg_db)) return pg_db;
    const float vref_adc = (cal.adc_vref_peak_volts > 0.0f)
        ? cal.adc_vref_peak_volts : 1.0f;
    return afe_dbfs + 20.0f * std::log10(vref_adc) - pg_db;
}

// Wenz natural ambient PSD at the preamp input, in dB ref 1 V/√Hz.
//   µPa/√Hz × 10^(rvr_db/20) = V/√Hz at preamp
//   In dB: psd_dbv_at_preamp = psd_db_uPa + rvr_db
// (`psd_db_uPa` is the noise PSD in dB re 1 µPa²/Hz, numerically equal to
// dB re 1 µPa/√Hz amplitude.)
[[nodiscard]] inline float dbv_at_preamp_from_db_uPa_per_sqrt_hz(
        float psd_db_uPa_per_sqrt_hz, float rvr_db) {
    return psd_db_uPa_per_sqrt_hz + rvr_db;
}

// Convert a target PSD at the preamp input (in dB ref 1 V/√Hz) to the
// receive-DAC sample dBFS that, when injected by NoiseGenerator, lands
// the noise at the preamp at the target level after passing through the
// modem's input attenuation pad and DAC voltage reference.
//
//   V_at_DAC = V_at_preamp / atten[op]_linear   (un-do the input pad)
//   DAC_sample = V_at_DAC / V_ref_dac_peak
// In dB:
//   dac_dbfs = preamp_dbv − input_atten[op]_dB − 20·log10(V_ref_dac_peak)
//
// (input_atten is in dB, negative for an attenuator; subtracting a
// negative adds the magnitude, which is the "drive harder than the desired
// preamp voltage" intuition.)
[[nodiscard]] inline float dac_dbfs_from_preamp_dbv_per_sqrt_hz(
        float                  target_dbv_at_preamp,
        const CalibrationData& cal,
        uint8_t                atten_idx) {
    const float atten_db = (atten_idx < 2)
        ? cal.input_attenuation[atten_idx]
        : 0.0f;
    const float vref_dac = (cal.dac_vref_peak_volts > 0.0f)
        ? cal.dac_vref_peak_volts : 1.0f;
    return target_dbv_at_preamp - atten_db - 20.0f * std::log10(vref_dac);
}

} // namespace openCREST::dsp
