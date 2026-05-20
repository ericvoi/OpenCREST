#pragma once
#include <cstdint>
#include <algorithm>

namespace openCREST {

// Convert an ADC raw sample to a normalised float in [-1, +1].
// The ADC uses unsigned offset-binary: midpoint (half-scale) maps to 0.0f.
inline float adc_to_float(uint16_t sample, uint8_t adc_bits) {
    const float midpoint = static_cast<float>(1u << (adc_bits - 1));
    return (static_cast<float>(sample) - midpoint) / midpoint;
}

// Convert a normalised float in [-1, +1] to a DAC raw sample.
// Clamps to the valid DAC range; never produces wrap-around artefacts.
inline uint16_t float_to_dac(float sample, uint8_t dac_bits) {
    const float midpoint = static_cast<float>(1u << (dac_bits - 1));
    const float max_val  = static_cast<float>((1u << dac_bits) - 1);
    const float scaled   = sample * midpoint + midpoint;
    const float clamped  = std::clamp(scaled, 0.0f, max_val);
    return static_cast<uint16_t>(clamped);
}

} // namespace openCREST
