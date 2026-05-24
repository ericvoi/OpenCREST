#pragma once
#include <cstddef>
#include <vector>

namespace openCREST::dsp {

// Discrete-time Hilbert transformer (Type-III odd-length FIR with anti-
// symmetric coefficients). Produces the imaginary part of the analytic
// signal x_a(t) = x(t) + j·x̂(t).
//
// Applies +90° phase shift to negative frequencies and −90° to positive
// (sign convention: x = cos(ω·n) → x̂ = sin(ω·n)) across the mid-band,
// with rolloff near DC and Nyquist set by `length` and the Hamming
// window on the truncated ideal coefficients.
//
// Output is delayed by group_delay_samples() = (length − 1) / 2 relative
// to input. To use the analytic signal coherently the caller must delay
// the real path by the same amount; process() emits both the delayed
// real samples and the imaginary samples in lockstep.
//
// Complex multipath tap g·e^(jφ) applied to a real bandpass input x
// produces real output g·(cos(φ)·x − sin(φ)·x̂); see
// Channel::scatter_taps complex-tap branch.
//
// State persists across process() calls. Allocation-free on the hot
// path; all storage sized at construction.
class HilbertFilter {
public:
    // `length` must be odd and ≥ 3. Default 31 taps yields ~3% transition
    // bands at DC and Nyquist with Hamming windowing — adequate for the
    // 25–35 kHz signal band at 500 kSPS.
    explicit HilbertFilter(size_t length = 31);

    // FIR group delay in samples. The caller must delay the real path by
    // this amount to keep the (real, imag) pair time-aligned.
    [[nodiscard]] size_t group_delay_samples() const { return group_delay_; }

    [[nodiscard]] size_t length() const { return length_; }

    // Process `n` input samples. Writes:
    //   delayed_real[0..n-1] — input delayed by group_delay_samples()
    //   hilbert_out[0..n-1]  — Hilbert transform of input, time-aligned
    //                          with delayed_real.
    // Buffers may not alias.
    void process(const float* input,
                 float* delayed_real,
                 float* hilbert_out,
                 size_t n);

    void reset();

private:
    size_t             length_;
    size_t             group_delay_;
    std::vector<float> coeffs_;     // length_ taps
    std::vector<float> history_;    // length_ samples, oldest at write_pos_
    size_t             write_pos_ = 0;
};

} // namespace openCREST::dsp
