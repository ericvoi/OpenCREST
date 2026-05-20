#pragma once

namespace openCREST::dsp {

// Thorp (1967) absorption coefficient.
// Returns α in dB/km for the given frequency.
//
//   α(f) = 0.11·f²/(1+f²)  +  44·f²/(4100+f²)  +  2.75e-4·f²  +  0.003
//
// where f is in kHz. Valid range: 0.1 – 100 kHz.
// For freshwater (saltwater = false) a simplified 0.002 dB/km/kHz² correction
// is subtracted at mid-frequencies (rough approximation).
float thorp_absorption_db_per_km(float freq_khz, bool saltwater = true);

// Total transmission loss (TL) in dB from source to receiver.
//   TL = k · log10(range_m) + α(freq_khz) · range_m / 1000
//
// k = spreading_factor (2.0 = spherical, 1.0 = cylindrical, 1.5 = hybrid)
float transmission_loss_db(float range_m,
                           float freq_khz,
                           float spreading_factor = 2.0f,
                           bool  saltwater        = true);

// Convenience: scalar linear gain (0 < g ≤ 1) for the given range/frequency.
// Equivalent to 10^(−TL / 20).
float path_loss_linear(float range_m,
                       float freq_khz,
                       float spreading_factor = 2.0f,
                       bool  saltwater        = true);

} // namespace openCREST::dsp
