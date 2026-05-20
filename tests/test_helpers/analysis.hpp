#pragma once
#include <vector>
#include <cstddef>

namespace openCREST::test {

// Root-mean-square of a signal.
float rms(const std::vector<float>& x);

// Signal-to-noise ratio (dB) between a reference and a distorted signal.
// SNR = 10 * log10( power(ref) / power(ref - test) )
float snr_db(const std::vector<float>& ref, const std::vector<float>& test);

// Peak absolute value.
float peak(const std::vector<float>& x);

// Estimate the dominant frequency of a signal via zero-crossing rate.
// Suitable for clean sinusoidal inputs.
float estimate_frequency_hz(const std::vector<float>& x, float sample_rate);

// Estimate the delay (in samples) between two signals using cross-correlation.
// Searches within [-max_lag, +max_lag] samples.
int estimate_delay_samples(const std::vector<float>& ref,
                           const std::vector<float>& delayed,
                           int max_lag);

// Return the power spectral density (linear) at `num_bins` uniformly-spaced
// frequency bins using a simple DFT magnitude squared / N.
// Bins correspond to frequencies k * sample_rate / N for k = 0..num_bins-1.
std::vector<float> power_spectrum(const std::vector<float>& x, size_t num_bins);

} // namespace openCREST::test
