#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace openCREST {

// Per-(source, receiver) sample buffer in absolute receiver-time.
//
// Carries the propagation-delayed and multipath-scattered signal from one
// source modem to one receiver modem. The source's SourceWorker scatters
// here; the receiver's ReceiverMix drains.
//
// Coordinate system
// -----------------
// Conceptually an infinite stream indexed by absolute receiver-time sample
// positions; physically a power-of-two ring of `capacity_` slots accessed
// via masking.
//
// Three positions:
//   * write_origin_    (producer-private) receiver-time index of the
//                      current message's source-sample-0 (zero tap delay).
//   * write_watermark_ (atomic release/acquire) committed boundary;
//                      consumer may read positions in [.., write_watermark_).
//   * read_pos_        (atomic release/acquire) consumer's read head.
//
// Multi-message semantics
// -----------------------
// The buffer is purely additive in absolute receiver time, so a source may
// begin a new message while the receiver still drains the prior one.
//
// First-message origin policy (selected by `absolute_first_origin` on
// begin_message()):
//   * false (PID-mode default): write_origin = base_delay_samples
//       — buffer encodes propagation delay structurally.
//   * true  (clock-tracker arrival-alignment): write_origin =
//       inter_message_gap_samples
//       — caller has already folded propagation delay into the gap; the
//       buffer must not double-shift.
//
// Subsequent messages (either policy):
//   write_origin = prior_watermark + inter_message_gap_samples
//
// SPSC contract
// -------------
// Producer:  begin_message, scatter_add, commit_source_progress,
//            commit_extra. (begin_message touches producer-private state
//            only; concurrent read_advance is safe.)
// Consumer:  read_advance.
// Either:    available_read, capacity, base_delay_samples, overflow_drops.
//
// scatter_add does NOT advance write_watermark_; only commit_* does. This
// hides multipath partial-state from the receiver during Channel::process's
// per-tap inner loop.
//
// Sizing (post power-of-two rounding):
//   capacity >= max_tap_delta_samples + max_in_flight_samples
// max_in_flight = base_delay + longest message + slack. ChannelEngine sizes
// from environment.max_range_m and environment.max_message_duration_s.
class PairBuffer {
public:
    PairBuffer(size_t capacity, size_t base_delay_samples);

    PairBuffer(const PairBuffer&)            = delete;
    PairBuffer& operator=(const PairBuffer&) = delete;
    PairBuffer(PairBuffer&&)                 = delete;
    PairBuffer& operator=(PairBuffer&&)      = delete;

    // ---------- Producer API (single source-worker thread) ----------

    // Source modem entered TX. Sets the new write_origin per the policy
    // documented at the class level. Producer-private only; concurrent
    // reads from the consumer side are safe.
    void begin_message(size_t inter_message_gap_samples = 0,
                       bool   absolute_first_origin    = false);

    // Accumulate `count` samples (+=) into receiver-time positions
    //   [write_origin + offset_samples, ... + count)
    // Channel computes offset as (source_position + tap_delta_k) per tap.
    // Samples beyond the consumer's read window are dropped and counted
    // in overflow_drops_.
    void scatter_add(size_t offset_samples, const float* samples, size_t count);

    // Advance write_watermark_ to write_origin + source_samples_processed
    // (monotonic; never moves backwards).
    void commit_source_progress(size_t source_samples_processed);

    // Extend the committed watermark by `extra_samples`. Channel uses
    // this in on_message_end to expose the multipath tail.
    void commit_extra(size_t extra_samples);

    // ---------- Consumer API (single receiver-mix thread) ----------

    // Read up to `count` samples into `out`; advance the read head by the
    // number returned. Consumed slots are zeroed so the producer may
    // scatter into them again after wrap-around. Capped at available_read()
    // so uncommitted samples are never returned.
    size_t read_advance(float* out, size_t count);

    // ---------- Diagnostics ----------

    size_t available_read() const;
    size_t capacity()           const { return capacity_; }
    size_t base_delay_samples() const { return base_delay_samples_; }
    uint64_t overflow_drops()   const {
        return overflow_drops_.load(std::memory_order_relaxed);
    }

    // Absolute receiver-time read position. Acquire-load pairs with the
    // consumer's release-store in read_advance. Safe from the producer
    // side for arrival-alignment math.
    size_t read_pos() const {
        return read_pos_.load(std::memory_order_acquire);
    }

    // Producer's published watermark in absolute receiver-time. Producer
    // may relaxed-load its own value; consumer uses acquire.
    size_t write_watermark() const {
        return write_watermark_.load(std::memory_order_acquire);
    }

private:
    std::unique_ptr<float[]> buffer_;
    size_t capacity_           = 0;   // power of two
    size_t mask_               = 0;   // capacity_ - 1
    size_t base_delay_samples_ = 0;

    // Consumer-owned cache line.
    alignas(64) std::atomic<size_t> read_pos_{0};

    // Producer-owned cache line.
    alignas(64) std::atomic<size_t> write_watermark_{0};

    // Producer-private state.
    size_t write_origin_              = 0;
    bool   first_message_             = true;

    std::atomic<uint64_t> overflow_drops_{0};
};

} // namespace openCREST
