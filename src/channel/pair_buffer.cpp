#include "channel/pair_buffer.hpp"
#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace openCREST {

namespace {
size_t round_up_pow2(size_t n) {
    if (n == 0) return 1;
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}
} // namespace

PairBuffer::PairBuffer(size_t capacity, size_t base_delay_samples)
    : capacity_(round_up_pow2(capacity))
    , mask_(capacity_ - 1)
    , base_delay_samples_(base_delay_samples)
{
    if (base_delay_samples_ >= capacity_) {
        throw std::invalid_argument(
            "PairBuffer: capacity must exceed base_delay_samples");
    }
    // Zero-init so the propagation-delay region reads as silence and any
    // slot a producer scatters into starts from 0.
    buffer_ = std::make_unique<float[]>(capacity_);
    std::memset(buffer_.get(), 0, capacity_ * sizeof(float));
}

// ---------------------------------------------------------------------------
// Producer API
// ---------------------------------------------------------------------------

void PairBuffer::begin_message(size_t inter_message_gap_samples,
                                bool   absolute_first_origin) {
    if (first_message_ && !absolute_first_origin) {
        // Legacy / PID-mode default: first message lands at base_delay so
        // the reader sees base_delay zeros before the message's sample 0.
        write_origin_ = base_delay_samples_;
        first_message_ = false;
        return;
    }
    // Unified path (clock-tracker arrival-alignment mode, or any subsequent
    // message): write_origin = prior_watermark + inter_message_gap_samples.
    // For the first message under absolute_first_origin=true, prior_watermark
    // == 0, so write_origin equals the caller-supplied gap. The caller has
    // already incorporated propagation delay into the gap (e.g. via
    // compute_arrival_aligned_gap), so the buffer must NOT auto-add base_delay
    // again — doing so was the source of the one-shot ~25 m ranging bias on
    // the first ranging after simulator restart, since the receiver-modem's
    // pre-fill backlog (~F_B samples) leaked through unaccounted-for.
    const size_t prior_watermark =
        write_watermark_.load(std::memory_order_relaxed);
    write_origin_   = prior_watermark + inter_message_gap_samples;
    first_message_  = false;
}

void PairBuffer::scatter_add(size_t offset_samples,
                              const float* samples,
                              size_t count) {
    if (count == 0) return;

    const size_t abs_start = write_origin_ + offset_samples;
    const size_t abs_end   = abs_start + count;

    // Overflow check against the consumer's read position. Producer's view
    // of read_pos is always <= the actual current read_pos (acquire on a
    // monotonically-advancing release store), so a stale view is conservative
    // and safe.
    const size_t read_pos = read_pos_.load(std::memory_order_acquire);
    const size_t max_writable_end = read_pos + capacity_;

    size_t to_write = count;
    if (abs_start >= max_writable_end) {
        // Entire write region is past the live window — drop all.
        overflow_drops_.fetch_add(count, std::memory_order_relaxed);
        return;
    }
    if (abs_end > max_writable_end) {
        const size_t dropped = abs_end - max_writable_end;
        overflow_drops_.fetch_add(dropped, std::memory_order_relaxed);
        to_write = max_writable_end - abs_start;
    }

    // Accumulate. Buffer slots reached here are guaranteed zero either from
    // initial allocation or from read_advance() zeroing on the consumer side.
    for (size_t i = 0; i < to_write; ++i) {
        buffer_[(abs_start + i) & mask_] += samples[i];
    }
}

void PairBuffer::commit_source_progress(size_t source_samples_processed) {
    const size_t new_watermark = write_origin_ + source_samples_processed;
    // write_watermark_ is producer-only-written, so relaxed-load is fine.
    const size_t cur = write_watermark_.load(std::memory_order_relaxed);
    if (new_watermark > cur) {
        write_watermark_.store(new_watermark, std::memory_order_release);
    }
}

void PairBuffer::commit_extra(size_t extra_samples) {
    if (extra_samples == 0) return;
    const size_t cur = write_watermark_.load(std::memory_order_relaxed);
    write_watermark_.store(cur + extra_samples, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Consumer API
// ---------------------------------------------------------------------------

size_t PairBuffer::read_advance(float* out, size_t count) {
    const size_t read_pos       = read_pos_.load(std::memory_order_relaxed);
    const size_t write_watermark = write_watermark_.load(std::memory_order_acquire);

    const size_t available = (write_watermark > read_pos)
        ? (write_watermark - read_pos) : 0;
    const size_t to_read = std::min(count, available);

    for (size_t i = 0; i < to_read; ++i) {
        const size_t slot = (read_pos + i) & mask_;
        out[i] = buffer_[slot];
        // Zero the slot so the producer's next scatter_add (after wrap)
        // accumulates from a clean slate.
        buffer_[slot] = 0.0f;
    }

    if (to_read > 0) {
        read_pos_.store(read_pos + to_read, std::memory_order_release);
    }
    return to_read;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

size_t PairBuffer::available_read() const {
    const size_t rp = read_pos_.load(std::memory_order_acquire);
    const size_t wm = write_watermark_.load(std::memory_order_acquire);
    return (wm > rp) ? (wm - rp) : 0;
}

} // namespace openCREST
