#include "dsp/delay_line.hpp"
#include <stdexcept>

namespace openCREST::dsp {

DelayLine::DelayLine(size_t capacity)
    : buffer_(capacity, 0.0f) {
    if (capacity == 0) {
        throw std::invalid_argument("DelayLine: capacity must be > 0");
    }
}

void DelayLine::write(float sample) {
    buffer_[write_pos_] = sample;
    if (++write_pos_ >= buffer_.size()) write_pos_ = 0;
}

void DelayLine::write_block(const float* samples, size_t count) {
    for (size_t i = 0; i < count; ++i) write(samples[i]);
}

float DelayLine::read(size_t delay_samples) const {
    // delay=0 → most recently written sample (just before write_pos_).
    const size_t sz  = buffer_.size();
    const size_t idx = (write_pos_ + sz - 1 - delay_samples) % sz;
    return buffer_[idx];
}

void DelayLine::reset() {
    std::fill(buffer_.begin(), buffer_.end(), 0.0f);
    write_pos_ = 0;
}

} // namespace openCREST::dsp
