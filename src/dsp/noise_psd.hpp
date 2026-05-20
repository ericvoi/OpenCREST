#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "config/scenario.hpp"
#include "core/types.hpp"
#include "dsp/calibration_math.hpp"
#include "dsp/wenz_model.hpp"

namespace openCREST::dsp {

// Phase C noise sizing. All functions are pure, header-only, and
// allocation-free; safe to call at scenario load. The hot path is
// untouched — these helpers compute scalars that downstream constructors
// (NoiseGenerator, Channel) consume.
//
// Reference frame for comparisons: the **preamp input**.
//
// Both natural ambient and AFE noise are expressed as one-sided amplitude
// PSD in dB ref 1 V/√Hz at the preamp input. The acoustic source naturally
// lives there (µPa × RVR → V at preamp); the AFE noise floor measured at
// the ADC is converted to the same point by un-doing the preamp gain.
// This keeps the comparison in `compute_boost_db` invariant to the input
// attenuation pad selection and to the DAC/ADC voltage references.
//
// The final NoiseGenerator target is then translated from preamp dBV back
// to receive-DAC sample dBFS, which is what the existing PSD → total-RMS
// machinery (`psd_target_to_total_rms_dbfs`) consumes.
//
// dB convention: amplitude PSD dB = 20·log10(amp_per_√Hz), numerically
// equal to 10·log10 of the one-sided power PSD per Hz (since power = amp²
// and 10·log10(amp²) = 20·log10(amp)).

// Wenz ambient noise PSD at the receiver's preamp input, in dB ref 1 V/√Hz.
//   µPa/√Hz @ fc_rx × 10^(rvr_db/20) = V/√Hz at preamp
//   In dB: result = noise_psd_db(fc_rx, …) + rvr_db
[[nodiscard]] inline float natural_noise_psd_dbv_at_preamp_per_sqrt_hz(
        float                  fc_rx_hz,
        const TransducerSpec&  receiver_transducer,
        int                    sea_state,
        bool                   saltwater) {
    const float wenz_amp_db_per_sqrt_hz_uPa =
        noise_psd_db(fc_rx_hz, sea_state, saltwater);
    return dbv_at_preamp_from_db_uPa_per_sqrt_hz(
        wenz_amp_db_per_sqrt_hz_uPa, receiver_transducer.rvr_db);
}

// Per-receiver auto-boost. The simulator's natural ambient noise PSD must
// dominate the modem's measured AFE PSD by `min_margin_db` so the
// simulation is physically meaningful. When natural is too quiet, scale
// up the noise (and every TX channel feeding this receiver) by `boost_db`
// — preserves SNR, costs clipping headroom.
//
// Both inputs MUST be in the same reference frame; the typical caller
// uses dB ref 1 V/√Hz at the preamp input.
[[nodiscard]] inline float compute_boost_db(
        float natural_psd_db,
        float afe_psd_db,
        float min_margin_above_afe_db = 10.0f) {
    if (!std::isfinite(afe_psd_db)) return 0.0f;
    const float required = afe_psd_db + min_margin_above_afe_db;
    if (!std::isfinite(natural_psd_db)) return 0.0f;
    return std::max(0.0f, required - natural_psd_db);
}

// Convert a target one-sided amplitude-PSD (dBFS/√Hz at fc) into the total
// RMS dBFS NoiseGenerator's existing API consumes. Accounts for the
// shaping FIR's response at fc and its energy.
//
// Derivation: y[n] = output_scale · (h * white)[n] with σ_white = 1.
//   one-sided PSD_y(fc) = output_scale² · |H(fc)|² · 2/Fs
//   total power_y       = output_scale² · Σh²
// Setting PSD_y(fc) to target_psd_per_hz and total_rms = NoiseGenerator's
// target_rms = output_scale · sqrt(Σh²):
//   target_rms = sqrt( target_psd_per_hz · Σh² · Fs / (2 · |H(fc)|²) )
[[nodiscard]] inline float psd_target_to_total_rms_dbfs(
        float                     target_psd_dbfs_per_sqrt_hz,
        const std::vector<float>& shaping_taps,
        float                     fc_hz,
        float                     fs_hz) {
    if (shaping_taps.empty() || fs_hz <= 0.0f) {
        return -std::numeric_limits<float>::infinity();
    }
    float energy = 0.0f;
    for (const float t : shaping_taps) energy += t * t;
    const float h_at_fc = filter_response_at(shaping_taps, fc_hz, fs_hz);
    if (energy <= 0.0f || h_at_fc <= 0.0f) {
        return -std::numeric_limits<float>::infinity();
    }
    return target_psd_dbfs_per_sqrt_hz
         + 10.0f * std::log10(energy)
         + 10.0f * std::log10(fs_hz / 2.0f)
         - 20.0f * std::log10(h_at_fc);
}

// One-shot per-receiver sizing aggregate. Computes natural Wenz PSD and
// AFE PSD at the preamp input (dB ref 1 V/√Hz), the boost needed to
// enforce `min_margin_above_afe_db` at that point, and the resulting
// receive-DAC sample dBFS that NoiseGenerator must achieve so the noise
// lands at the preamp at `(natural + boost)`.
struct ReceiverNoiseSizing {
    // Wenz ambient PSD at the preamp input, dB ref 1 V/√Hz (one-sided amp).
    float natural_psd_dbv_at_preamp = 0.0f;
    // AFE noise floor input-referred to the preamp, dB ref 1 V/√Hz. Derived
    // from cal.noise_floor_psd_counts_per_sqrt_hz, V_ref_adc, and the
    // preamp gain (from cal.loopback_gain & cal.loopback_cal_attenuation).
    // Falls back to kFallbackAfePsdDbvAtPreamp when uncalibrated.
    float afe_psd_dbv_at_preamp     = 0.0f;
    // dB by which natural is below (afe + margin); zero if natural already
    // dominates. Applied identically to noise injection and to every
    // channel feeding this receiver, so SNR is preserved.
    float boost_db                  = 0.0f;
    // What `psd_target_to_total_rms_dbfs` consumes: target one-sided amp
    // PSD at fc, in receive-DAC sample dBFS. Derived from
    // `(natural + boost)` at the preamp by un-doing the input attenuation
    // pad and the DAC voltage reference.
    float target_psd_dbfs_at_dac    = 0.0f;
};

// Default fallback AFE PSD (preamp input, dB ref 1 V/√Hz) when the modem
// has not reported a calibrated noise floor (uncalibrated →
// noise_floor_psd_counts_per_sqrt_hz == 0). The number is intentionally
// quiet so an uncalibrated modem doesn't trigger spurious boosts.
constexpr float kFallbackAfePsdDbvAtPreamp = -220.0f;

[[nodiscard]] inline ReceiverNoiseSizing compute_receiver_noise_sizing(
        const CalibrationData& receiver_cal,
        const TransducerSpec&  receiver_transducer,
        int                    sea_state,
        bool                   saltwater,
        std::uint8_t           atten_idx,
        float                  min_margin_above_afe_db) {
    ReceiverNoiseSizing s{};
    s.natural_psd_dbv_at_preamp = natural_noise_psd_dbv_at_preamp_per_sqrt_hz(
        receiver_cal.center_freq_hz, receiver_transducer, sea_state, saltwater);

    const float afe_raw = afe_psd_dbv_at_preamp_per_sqrt_hz(receiver_cal);
    s.afe_psd_dbv_at_preamp =
        std::isfinite(afe_raw) ? afe_raw : kFallbackAfePsdDbvAtPreamp;

    s.boost_db = compute_boost_db(s.natural_psd_dbv_at_preamp,
                                  s.afe_psd_dbv_at_preamp,
                                  min_margin_above_afe_db);
    const float target_dbv_at_preamp =
        s.natural_psd_dbv_at_preamp + s.boost_db;
    s.target_psd_dbfs_at_dac = dac_dbfs_from_preamp_dbv_per_sqrt_hz(
        target_dbv_at_preamp, receiver_cal, atten_idx);
    return s;
}

} // namespace openCREST::dsp
