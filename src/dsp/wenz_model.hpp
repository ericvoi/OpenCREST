#pragma once
#include <vector>
#include <cstddef>

namespace openCREST::dsp {

// Wenz (1962) ambient noise model for underwater acoustics.
//
// All functions work in physical units:
//   freq_hz  — frequency in Hz
//   sea_state — 0 (glassy) to 6 (very rough), Beaufort scale equivalent
//   saltwater — true for ocean, false for fresh water (affects thermal term)
//
// PSD values are in dB re 1 µPa²/Hz (standard underwater acoustics reference).

// Approximate wind speed (knots) for a given Beaufort sea state.
float sea_state_to_wind_knots(int sea_state);

// Wind/sea-state driven noise spectral density (dominant 100 Hz – 50 kHz).
// Implements the Wenz surface-agitation formula:
//   N_w(f) = 50 + 7.5·√w + 20·log10(f_kHz) − 40·log10(f_kHz + 0.4)   [dB re µPa²/Hz]
float wind_noise_psd_db(float freq_hz, int sea_state);

// Thermal (molecular) noise spectral density (dominant above ~50 kHz).
//   N_th(f) = −15 + 20·log10(f_kHz)   [dB re µPa²/Hz]
float thermal_noise_psd_db(float freq_hz);

// Total ambient noise PSD (incoherent sum of wind + thermal components).
float noise_psd_db(float freq_hz, int sea_state, bool saltwater = true);

// Design a Wenz-shaped FIR coloring filter.
//
// Returns `n_taps` coefficients (Type I, symmetric) designed by the
// windowed frequency-sampling method. The filter magnitude response
// approximates the square-root of the Wenz PSD (coloring a white-noise
// input yields the correct power spectrum). Normalised so the peak
// coefficient response equals 1.0.
//
// Typical use: n_taps = 32, applied to unit-variance Gaussian white noise.
std::vector<float> design_shaping_filter(float sample_rate_hz,
                                         int   sea_state,
                                         bool  saltwater = true,
                                         size_t n_taps   = 32);

// Magnitude response |H(fc)| of a real FIR `taps` evaluated at frequency
// `fc_hz` for sample rate `fs_hz`. Direct DFT-bin evaluation; O(N)
// mul-adds. Returns 0 for degenerate inputs (empty taps, fs ≤ 0). Used at
// scenario load to convert a target noise PSD at the receiver's center
// frequency into the total-RMS target consumable by NoiseGenerator.
float filter_response_at(const std::vector<float>& taps,
                         float fc_hz, float fs_hz);

} // namespace openCREST::dsp
