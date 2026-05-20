#pragma once
#include <cstddef>
#include <vector>

namespace openCREST::dsp {

// Circular-buffer delay line.
//
// write() appends one sample; write_block() appends N samples.
// read(delay) returns the sample `delay` steps before the most recently
// written one (delay=0 → most recent, delay=capacity-1 → oldest valid).
// All pre-write positions are initialised to 0.0f.
class DelayLine {
public:
    explicit DelayLine(size_t capacity);

    void  write(float sample);
    void  write_block(const float* samples, size_t count);

    float read(size_t delay_samples) const;

    void   reset();
    size_t capacity() const { return buffer_.size(); }

private:
    std::vector<float> buffer_;
    size_t             write_pos_ = 0;
};

} // namespace openCREST::dsp
