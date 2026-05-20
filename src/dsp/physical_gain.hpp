#pragma once
#include <cmath>
#include <cstdint>

#include "config/scenario.hpp"
#include "core/types.hpp"
#include "dsp/path_loss.hpp"

namespace openCREST::dsp {

// Per-channel physically-derived gain (dB). Chains the source-side voltage
// anchor + TX monitor pad + TVR, the acoustic path loss at the source's
// center frequency, and the receiver-side RVR + RX injection pad + DAC
// voltage anchor.
//
//   physical_gain_dB =
//         20·log10(V_ref_adc_src_peak)            // ADC FS in V
//       + (-output_atten_src_dB)                  // un-do TX monitor pad
//       + tvr_src_db                              // V_drive → µPa @ 1 m
//       - TL_dB(range, src_cal.center_freq_hz)    // Thorp at source's freq
//       + rvr_rx_db                               // µPa → V at preamp
//       + (-input_atten_rx[atten_idx]_dB)         // un-do RX injection pad
//       - 20·log10(V_ref_dac_rx_peak)             // V_DAC → sample
//
// With default CalibrationData{} (Vref = 1.0 V, attenuations = 0 dB,
// fc = 25 kHz) and TransducerSpec{} (TVR = RVR = 0 dB) this collapses to
// -TL_dB at the default center frequency, preserving today's behavior for
// fixtures that don't supply real calibration data.
//
// The result is in dB-amplitude (20·log10 of the linear scalar that maps
// a source ADC sample to the corresponding receiver DAC sample). Pure,
// allocation-free, safe to call at scenario load.
[[nodiscard]] inline double compute_channel_physical_gain_db(
        const CalibrationData& src_cal,
        const CalibrationData& rx_cal,
        const TransducerSpec&  src_tx,
        const TransducerSpec&  rx_tx,
        const ChannelConfig&   cfg,
        std::uint8_t           atten_idx) {
    const double v_adc = (src_cal.adc_vref_peak_volts > 0.0f)
        ? static_cast<double>(src_cal.adc_vref_peak_volts) : 1.0;
    const double v_dac = (rx_cal.dac_vref_peak_volts > 0.0f)
        ? static_cast<double>(rx_cal.dac_vref_peak_volts) : 1.0;

    const double out_atten = static_cast<double>(src_cal.output_attenuation);
    const double in_atten  = (atten_idx < 2)
        ? static_cast<double>(rx_cal.input_attenuation[atten_idx])
        : 0.0;

    const double fc_khz = static_cast<double>(src_cal.center_freq_hz) / 1000.0;
    const double tl_db  = static_cast<double>(transmission_loss_db(
        cfg.range_m, static_cast<float>(fc_khz),
        cfg.spreading_factor, cfg.saltwater));

    return 20.0 * std::log10(v_adc)
         + (-out_atten)
         + static_cast<double>(src_tx.tvr_db)
         - tl_db
         + static_cast<double>(rx_tx.rvr_db)
         + (-in_atten)
         - 20.0 * std::log10(v_dac);
}

// AFE-electrical chain only — same formula as `compute_channel_physical_gain_db`
// minus the −TL term. Used by geometric-mode Channel where the geometric scene
// supplies the per-path spreading + Thorp factor (eq. 2) and only the
// frequency-independent electronics chain (V_ref, TX/RX pads, TVR, RVR)
// multiplies on top.
[[nodiscard]] inline double compute_channel_afe_chain_gain_db(
        const CalibrationData& src_cal,
        const CalibrationData& rx_cal,
        const TransducerSpec&  src_tx,
        const TransducerSpec&  rx_tx,
        std::uint8_t           atten_idx) {
    const double v_adc = (src_cal.adc_vref_peak_volts > 0.0f)
        ? static_cast<double>(src_cal.adc_vref_peak_volts) : 1.0;
    const double v_dac = (rx_cal.dac_vref_peak_volts > 0.0f)
        ? static_cast<double>(rx_cal.dac_vref_peak_volts) : 1.0;

    const double out_atten = static_cast<double>(src_cal.output_attenuation);
    const double in_atten  = (atten_idx < 2)
        ? static_cast<double>(rx_cal.input_attenuation[atten_idx])
        : 0.0;

    return 20.0 * std::log10(v_adc)
         + (-out_atten)
         + static_cast<double>(src_tx.tvr_db)
         + static_cast<double>(rx_tx.rvr_db)
         + (-in_atten)
         - 20.0 * std::log10(v_dac);
}

} // namespace openCREST::dsp
