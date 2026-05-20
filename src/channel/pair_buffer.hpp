#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace openCREST {

// Per-(source, receiver) sample buffer in absolute receiver-time.
//
// One PairBuffer carries the propagation-delayed and multipath-scattered
// signal from a single source modem to a single receiver modem. The
// SourceWorker for the source modem scatters-add samples here; the
// ReceiverMix for the destination modem drains them.
//
// Coordinate system
// -----------------
// The buffer is conceptually an infinite stream indexed by absolute
// receiver-time sample positions (uint64-style monotonic counters).
// Internally, it is a power-of-two ring of `capacity_` slots; absolute
// positions wrap onto physical slots via masking.
//
// Three positions matter:
//   * write_origin_       (producer-private) — receiver-time index of the
//                         current message's source-sample-0 (zero tap delay).
//   * write_watermark_    (atomic, release/acquire) — committed boundary;
//                         consumer may read positions in [.., write_watermark_).
//   * read_pos_           (atomic, release/acquire) — consumer's read head.
//
// Multi-message semantics (matters for back-to-back transmissions)
// ----------------------------------------------------------------
// The architecture must allow a source modem to begin a new TX message while
// the receiver is still draining the previous one. Therefore the buffer is
// purely additive in absolute receiver time.
//
// Two policies for the FIRST message after construction, selected via the
// `absolute_first_origin` parameter to begin_message():
//
//   * absolute_first_origin = false  (legacy / PID-mode default):
//         write_origin = base_delay_samples
//         (source sample 0 arrives at receiver index base_delay_samples;
//         reader sees base_delay zeros first — propagation delay encoded
//         purely by the buffer)
//   * absolute_first_origin = true   (clock-tracker arrival-alignment):
//         write_origin = inter_message_gap_samples
//         (caller has already incorporated propagation delay into the gap
//         via the arrival-alignment formula, which also subtracts
//         pipeline-depth / receiver-fill contributions; the buffer must
//         not double-shift)
//
// On each SUBSEQUENT message (regardless of policy):
//         new write_origin = prior write_watermark + inter_message_gap_samples
//
// inter_message_gap_samples expresses source-time idle between the end of
// the prior message and the start of the new one (in source-clock samples).
// For Phase 2 MVP PID callers, may pass 0 (back-to-back). The receiver then
// reads the prior message's data, naturally continues into the new
// message's data, and never observes corruption.
//
// Why the policy parameter exists: pre-fix, the first message ALWAYS
// auto-applied base_delay and IGNORED inter_message_gap_samples. In
// clock-tracker mode that bypassed arrival-alignment entirely, so the
// receiver's pre-fill backlog (~F_B samples) became a one-shot ~25 m
// ranging bias on the first ranging after simulator restart.
//
// SPSC contract
// -------------
// Producer thread:  begin_message, scatter_add, commit_source_progress,
//                   commit_extra. (begin_message updates producer-private
//                   state only; safe with concurrent read_advance.)
// Consumer thread:  read_advance.
// Either thread:    available_read, capacity, base_delay_samples,
//                   overflow_drops.
//
// scatter_add does NOT advance write_watermark_ — only commit_* does. This
// keeps multipath partial-state hidden from the receiver during the
// per-tap inner loop in Channel::process.
//
// Sizing
// ------
// capacity (rounded up to next power of two) must satisfy:
//   capacity >= max_tap_delta_samples + max_in_flight_samples
// where max_in_flight = base_delay + (longest message + slack). For Phase 2
// the simulator sizes capacity from a worst-case `environment.max_range_m`
// and a configured maximum message duration.
class PairBuffer {
public:
    PairBuffer(size_t capacity, size_t base_delay_samples);

    PairBuffer(const PairBuffer&)            = delete;
    PairBuffer& operator=(const PairBuffer&) = delete;
    PairBuffer(PairBuffer&&)                 = delete;
    PairBuffer& operator=(PairBuffer&&)      = delete;

    // -----------------------------------------------------------------------
    // Producer API (single source-worker thread)
    // -----------------------------------------------------------------------

    // Source modem entered TX. Sets the new write_origin (see "Multi-message
    // semantics" above). `absolute_first_origin = true` selects the
    // clock-tracker arrival-alignment policy on the first message (caller's
    // gap is the absolute receiver-time offset and already incorporates
    // propagation delay). Default false preserves the legacy PID-mode
    // behavior of auto-applying base_delay_samples_ on the first message.
    //
    // Producer-private only — does not touch read_pos_ or write_watermark_.
    // Therefore concurrent reads from the consumer side are safe; the
    // consumer continues draining the prior message's data through
    // write_watermark_ and naturally observes the new message's data once
    // commit_source_progress publishes it.
    void begin_message(size_t inter_message_gap_samples = 0,
                       bool   absolute_first_origin    = false);

    // Accumulate `count` samples into receiver-time positions
    //   [write_origin + offset_samples, write_origin + offset_samples + count)
    // The offset is computed by Channel as (source_position + tap_delta_k)
    // for each multipath tap k. Uses += semantics so independent taps and
    // overlapping messages overlap correctly.
    //
    // If the write region would overrun the consumer's read_pos (sizing bug),
    // the offending samples are dropped and overflow_drops_ is incremented.
    void scatter_add(size_t offset_samples, const float* samples, size_t count);

    // Publish source positions [0, source_samples_processed) of the current
    // message — i.e., advance write_watermark_ to
    //   write_origin + source_samples_processed
    // (only if greater than current watermark; never moves backwards).
    // source_samples_processed is monotonic-non-decreasing within a message.
    void commit_source_progress(size_t source_samples_processed);

    // Extend the published watermark by `extra_samples` past the current
    // commit. Used by Channel::on_message_end to expose the multipath tail
    // (extra_samples = max_tap_delta_samples). May also be used to expose
    // the trailing Farrow interpolation tail.
    void commit_extra(size_t extra_samples);

    // -----------------------------------------------------------------------
    // Consumer API (single receiver-mix thread)
    // -----------------------------------------------------------------------

    // Read up to `count` samples from current read head into `out`; advance
    // read head by the number returned. Each consumed slot is zeroed so the
    // producer may scatter-add into it again after wrap-around. Returns the
    // count actually read; capped at available_read() so the consumer never
    // reads uncommitted samples.
    size_t read_advance(float* out, size_t count);

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    size_t available_read() const;
    size_t capacity()           const { return capacity_; }
    size_t base_delay_samples() const { return base_delay_samples_; }
    uint64_t overflow_drops()   const {
        return overflow_drops_.load(std::memory_order_relaxed);
    }

    // Absolute receiver-time read position (consumer's current head).
    // Acquire-load: pairs with the consumer's release-store in
    // read_advance. Safe to call from the producer side for use in
    // arrival-alignment math (clock-extrapolation mode).
    size_t read_pos() const {
        return read_pos_.load(std::memory_order_acquire);
    }

    // Absolute receiver-time committed boundary (producer's published
    // watermark). Producer may relaxed-load its own value; consumer
    // uses acquire. Both are fine for arrival-alignment math which
    // tolerates ±sample-budget slop.
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
