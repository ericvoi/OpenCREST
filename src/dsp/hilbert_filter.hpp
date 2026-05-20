#pragma once
#include <cstddef>
#include <vector>

namespace openCREST::dsp {

// Discrete-time Hilbert transformer (Type-III odd-length FIR with anti-
// symmetric coefficients). Produces the imaginary part of the analytic
// signal x_a(t) = x(t) + j·x̂(t) where x̂ is the Hilbert transform of x.
//
// The filter applies a constant +90° phase shift to negative frequencies and
// −90° to positive frequencies (sign convention: x = cos(ω·n) → x̂ = sin(ω·n))
// across the mid-band, with rolloff near DC and Nyquist that depends on
// `length` and the Hamming window applied to the truncated ideal coefficients.
//
// The output is delayed by `group_delay_samples()` = (length − 1) / 2 samples
// relative to the input. To use the analytic signal coherently the caller
// must delay the *real* path by the same amount; `process()` produces both
// the delayed real samples and the imaginary samples in one shot so the two
// outputs are time-aligned at the caller's index.
//
// MVP usage: a complex multipath tap `g·e^(jφ)` applied to a real bandpass
// input x produces real output `g·(cos(φ)·x − sin(φ)·x̂)`. See the
// `Channel::scatter_taps` complex-tap branch.
//
// State persists across `process()` calls so consecutive blocks of input
// produce a continuous output stream. Allocation-free on the hot path —
// all state lives in vectors sized at construction.
class HilbertFilter {
public:
    // `length` must be odd and ≥ 3. Default 31 taps yields ~3% transition
    // bands at DC and Nyquist with Hamming windowing — adequate for the
    // 25–35 kHz signal band at 500 kSPS.
    explicit HilbertFilter(size_t length = 31);

    // Group delay introduced by the FIR (in samples). The caller must
    // delay the real path by this amount to keep the (real, imag) pair
    // time-aligned.
    [[nodiscard]] size_t group_delay_samples() const { return group_delay_; }

    [[nodiscard]] size_t length() const { return length_; }

    // Process `n` input samples. Writes:
    //   delayed_real[0..n-1] — input delayed by group_delay_samples()
    //   hilbert_out[0..n-1]  — Hilbert transform of input, time-aligned with
    //                          delayed_real (i.e. both refer to the same
    //                          input instant after one group delay).
    // Buffers may not alias.
    void process(const float* input,
                 float* delayed_real,
                 float* hilbert_out,
                 size_t n);

    // Reset the filter's history to all zeros.
    void reset();

private:
    size_t             length_;
    size_t             group_delay_;
    std::vector<float> coeffs_;     // length_ taps
    std::vector<float> history_;    // length_ samples, oldest at write_pos_
    size_t             write_pos_ = 0;
};

} // namespace openCREST::dsp
