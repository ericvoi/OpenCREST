#pragma once
#include <vector>
#include <cstddef>

namespace openCREST::dsp {

// Wenz (1962) ambient noise model for underwater acoustics.
//
// Parameters:
//   freq_hz   — frequency in Hz
//   sea_state — 0 (glassy) to 6 (very rough), Beaufort equivalent
//   saltwater — ocean vs fresh water (affects thermal term)
// PSD values are dB re 1 µPa²/Hz.

// Approximate wind speed (knots) for a given Beaufort sea state.
float sea_state_to_wind_knots(int sea_state);

// Wind/sea-state noise PSD (dominant 100 Hz – 50 kHz):
//   N_w(f) = 50 + 7.5·√w + 20·log10(f_kHz) − 40·log10(f_kHz + 0.4)
float wind_noise_psd_db(float freq_hz, int sea_state);

// Thermal (molecular) noise PSD (dominant above ~50 kHz):
//   N_th(f) = −15 + 20·log10(f_kHz)
float thermal_noise_psd_db(float freq_hz);

// Total ambient noise PSD (incoherent sum of wind + thermal).
float noise_psd_db(float freq_hz, int sea_state, bool saltwater = true);

// Design a Wenz-shaped FIR coloring filter.
//
// Returns `n_taps` Type-I symmetric coefficients via windowed frequency
// sampling. Magnitude response ≈ √(Wenz PSD), so colouring a white-noise
// input yields the correct power spectrum. Normalised to peak = 1.0.
//
// Typical use: n_taps = 32 applied to unit-variance Gaussian white noise.
std::vector<float> design_shaping_filter(float sample_rate_hz,
                                         int   sea_state,
                                         bool  saltwater = true,
                                         size_t n_taps   = 32);

// |H(fc)| of a real FIR `taps` evaluated at frequency `fc_hz` for sample
// rate `fs_hz`. Direct O(N) DFT-bin evaluation. Returns 0 for degenerate
// inputs (empty taps, fs ≤ 0). Used at scenario load to convert a target
// noise PSD at the receiver's centre frequency into the total-RMS target
// NoiseGenerator consumes.
float filter_response_at(const std::vector<float>& taps,
                         float fc_hz, float fs_hz);

} // namespace openCREST::dsp
