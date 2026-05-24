#pragma once

namespace openCREST::dsp {

// Thorp (1967) absorption coefficient in dB/km.
//   α(f) = 0.11·f²/(1+f²) + 44·f²/(4100+f²) + 2.75e-4·f² + 0.003
// f in kHz; valid 0.1 – 100 kHz. Freshwater (saltwater=false) subtracts
// a simplified mid-frequency correction.
float thorp_absorption_db_per_km(float freq_khz, bool saltwater = true);

// Total transmission loss (dB) source → receiver.
//   TL = k · log10(range_m) + α(freq_khz) · range_m / 1000
// k = spreading_factor (2.0 spherical, 1.0 cylindrical, 1.5 hybrid).
float transmission_loss_db(float range_m,
                           float freq_khz,
                           float spreading_factor = 2.0f,
                           bool  saltwater        = true);

// Scalar linear gain (0 < g ≤ 1) = 10^(−TL/20).
float path_loss_linear(float range_m,
                       float freq_khz,
                       float spreading_factor = 2.0f,
                       bool  saltwater        = true);

} // namespace openCREST::dsp
