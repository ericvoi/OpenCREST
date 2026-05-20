#pragma once
#include <cmath>
#include <cstddef>

namespace openCREST::dsp {

// Frequency-dependent transducer response (TVR on transmit, RVR on
// receive). MVP ships a frequency-flat implementation; a future FIR-based
// implementation can be substituted at every call site without source
// changes.
//
// Ordering when chained with Doppler:
//   - On the source side, apply TVR *before* the Farrow resampler. The
//     transducer's frequency response acts on the radiated waveform; the
//     Doppler shift then operates on the already-coloured spectrum.
//   - On the receiver side, apply RVR *after* the Farrow resampler. The
//     received pressure is already Doppler-shifted at the listener; the
//     transducer's response colours that spectrum.
//   For a flat (frequency-independent) response the order is irrelevant
//   because constant gain commutes with resampling.
class TransducerResponse {
public:
    virtual ~TransducerResponse();

    // Effective gain (dB) at a single frequency. For frequency-flat
    // responses this is constant; for FIR-based responses this reads the
    // |H(f)| at the given frequency.
    [[nodiscard]] virtual float gain_db_at(float freq_hz) const = 0;

    // Apply the response to `n` consecutive samples in-place-friendly form
    // (`out` may alias `in`). For a flat response this multiplies by a
    // precomputed linear gain. For a FIR response this convolves the
    // input with the impulse response (state-bearing impls must declare
    // their own state-management contract).
    virtual void apply(const float* in, float* out, std::size_t n) const = 0;
};

// Frequency-flat response — single dB scalar. The constructor caches the
// linear gain so apply() is a tight multiply loop, and gain_db_at()
// returns the configured dB value at any frequency.
class FlatResponse final : public TransducerResponse {
public:
    explicit FlatResponse(float gain_db) noexcept
        : gain_db_(gain_db),
          gain_lin_(std::pow(10.0f, gain_db / 20.0f)) {}

    [[nodiscard]] float gain_db_at(float /*freq_hz*/) const override {
        return gain_db_;
    }

    void apply(const float* in, float* out, std::size_t n) const override {
        for (std::size_t i = 0; i < n; ++i) {
            out[i] = in[i] * gain_lin_;
        }
    }

    [[nodiscard]] float gain_lin() const noexcept { return gain_lin_; }

private:
    float gain_db_;
    float gain_lin_;
};

} // namespace openCREST::dsp
