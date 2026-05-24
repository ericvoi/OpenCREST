#pragma once
#include <cmath>
#include <cstddef>

namespace openCREST::dsp {

// Frequency-dependent transducer response (TVR on transmit, RVR on
// receive). Currently a frequency-flat implementation; a FIR-based
// implementation can be substituted at every call site without source
// changes.
//
// Ordering when chained with Doppler:
//   - Source side: apply TVR *before* the Farrow resampler. The
//     transducer colours the radiated waveform; Doppler then shifts the
//     already-coloured spectrum.
//   - Receiver side: apply RVR *after* the Farrow resampler. Received
//     pressure is already Doppler-shifted; the transducer colours that.
// For a flat response the order is irrelevant.
class TransducerResponse {
public:
    virtual ~TransducerResponse();

    // Effective gain (dB) at a single frequency.
    [[nodiscard]] virtual float gain_db_at(float freq_hz) const = 0;

    // Apply the response to `n` consecutive samples. `out` may alias
    // `in`. Flat: multiply. FIR: convolve (state-bearing impls must
    // declare their own state contract).
    virtual void apply(const float* in, float* out, std::size_t n) const = 0;
};

// Frequency-flat response — single dB scalar; constructor caches the
// linear gain so apply() is a tight multiply loop.
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
