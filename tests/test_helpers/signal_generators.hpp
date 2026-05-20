#pragma once
#include <vector>
#include <cstddef>

namespace openCREST::test {

// Generate a sinusoidal tone: amplitude * sin(2π * freq_hz * n / sample_rate)
std::vector<float> make_tone(float freq_hz, float amplitude,
                              float sample_rate, size_t num_samples);

// Generate a linear frequency-sweep (chirp) from freq_start_hz to freq_end_hz.
std::vector<float> make_chirp(float freq_start_hz, float freq_end_hz,
                               float amplitude, float sample_rate,
                               size_t num_samples);

// Kronecker delta at position `offset` with amplitude `amplitude`.
std::vector<float> make_impulse(size_t num_samples, size_t offset = 0,
                                 float amplitude = 1.0f);

// DC (constant) signal.
std::vector<float> make_dc(float value, size_t num_samples);

} // namespace openCREST::test
