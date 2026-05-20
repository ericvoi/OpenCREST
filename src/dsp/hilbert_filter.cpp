#include "dsp/hilbert_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace openCREST::dsp {

namespace {

// Ideal anti-symmetric Hilbert FIR (Type III) — even-indexed coefficients
// are zero, odd-indexed coefficients are 2/(π·k). Centred at index
// `(length−1)/2`, multiplied by a Hamming window for a finite design.
std::vector<float> design_hilbert_taps(size_t length) {
    if ((length % 2) == 0 || length < 3) {
        throw std::invalid_argument(
            "HilbertFilter length must be odd and ≥ 3");
    }
    std::vector<float> h(length, 0.0f);
    const long center = static_cast<long>(length / 2);
    for (long n = 0; n < static_cast<long>(length); ++n) {
        const long k = n - center;
        if (k == 0 || (k % 2) == 0) {
            h[static_cast<size_t>(n)] = 0.0f;
        } else {
            const double ideal = 2.0 / (M_PI * static_cast<double>(k));
            // Hamming window over the full filter length.
            const double w = 0.54 - 0.46 * std::cos(
                2.0 * M_PI * static_cast<double>(n) /
                static_cast<double>(length - 1));
            h[static_cast<size_t>(n)] = static_cast<float>(ideal * w);
        }
    }
    return h;
}

} // namespace

HilbertFilter::HilbertFilter(size_t length)
    : length_(length)
    , group_delay_((length - 1) / 2)
    , coeffs_(design_hilbert_taps(length))
    , history_(length, 0.0f)
    , write_pos_(0)
{}

void HilbertFilter::reset() {
    std::fill(history_.begin(), history_.end(), 0.0f);
    write_pos_ = 0;
}

void HilbertFilter::process(const float* input,
                             float* delayed_real,
                             float* hilbert_out,
                             size_t n) {
    const size_t L = length_;
    // The delayed-real output for input index i is the input at i − D
    // where D = group_delay_. With our circular history buffer, that sample
    // is the value at slot `(write_pos_ + (L - D)) & (L-1)` if L is a power
    // of two — but here L is odd, so use modular arithmetic.
    //
    // Convention: history_[write_pos_] holds the current input; history_
    // wraps modulo L. The convolution sum reads
    //   y[i] = Σ_{k=0..L-1} coeffs_[k] · history_[(write_pos_ + L - k) % L]
    // which is the standard FIR formulation. The delayed-real output is
    // simply history_[(write_pos_ + L - center) % L] where center = D.

    for (size_t i = 0; i < n; ++i) {
        history_[write_pos_] = input[i];

        // Compute Hilbert (FIR convolution).
        double acc = 0.0;
        for (size_t k = 0; k < L; ++k) {
            const float c = coeffs_[k];
            if (c == 0.0f) continue;   // skip even taps fast (~half the loop)
            const size_t idx = (write_pos_ + L - k) % L;
            acc += static_cast<double>(c) * history_[idx];
        }
        hilbert_out[i] = static_cast<float>(acc);

        // Delayed real path.
        const size_t delay_idx = (write_pos_ + L - group_delay_) % L;
        delayed_real[i] = history_[delay_idx];

        // Advance ring head.
        write_pos_ = (write_pos_ + 1) % L;
    }
}

} // namespace openCREST::dsp
