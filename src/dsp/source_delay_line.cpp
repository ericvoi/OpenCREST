#include "dsp/source_delay_line.hpp"
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace openCREST::dsp {

namespace {

size_t round_up_pow2(size_t n) {
    if (n == 0) return 1;
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

} // namespace

void SourceDelayLine::resize(size_t capacity_samples) {
    const size_t cap = round_up_pow2(capacity_samples);
    if (cap == 0) {
        throw std::invalid_argument(
            "SourceDelayLine: capacity must be > 0");
    }
    buffer_.assign(cap, 0.0f);
    mask_              = cap - 1;
    producer_position_ = 0;
}

void SourceDelayLine::clear() {
    std::memset(buffer_.data(), 0, buffer_.size() * sizeof(float));
    producer_position_ = 0;
}

void SourceDelayLine::write(const float* in, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        buffer_[(producer_position_ + i) & mask_] = in[i];
    }
    producer_position_ += n;
}

float SourceDelayLine::read_at(double pos) const {
    // Catmull-Rom / Hermite cubic interpolation, same Horner form as
    // FarrowResampler::interpolate. Reads x[n-1], x[n], x[n+1], x[n+2]
    // from the ring where n = floor(pos).
    const double n_floor = std::floor(pos);
    const float  mu      = static_cast<float>(pos - n_floor);
    const int64_t n      = static_cast<int64_t>(n_floor);

    // Cast to uint64_t for the mask; for n ≥ 1 (the safe range)
    // the values are positive and the masking is straightforward.
    const uint64_t i0 = static_cast<uint64_t>(n - 1) & mask_;
    const uint64_t i1 = static_cast<uint64_t>(n)     & mask_;
    const uint64_t i2 = static_cast<uint64_t>(n + 1) & mask_;
    const uint64_t i3 = static_cast<uint64_t>(n + 2) & mask_;

    const float h0 = buffer_[i0];
    const float h1 = buffer_[i1];
    const float h2 = buffer_[i2];
    const float h3 = buffer_[i3];

    return h1 + 0.5f * mu * ((h2 - h0)
        + mu * ((2.0f * h0 - 5.0f * h1 + 4.0f * h2 - h3)
            + mu * (3.0f * (h1 - h2) + h3 - h0)));
}

} // namespace openCREST::dsp
