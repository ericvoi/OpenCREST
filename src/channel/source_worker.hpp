#pragma once
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "core/ring_buffer.hpp"
#include "core/types.hpp"
#include "core/fill_tracker.hpp"
#include "core/tx_start_estimator.hpp"
#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "simulator/metrics.hpp"
#include "simulator/message_event_log.hpp"
#include "simulator/processing_time_stats.hpp"

namespace openCREST {

// One thread per source modem.
//
// Drains the source modem's tx_ring, converts ADC samples to float, and
// hands the chunk to each outgoing Channel which in turn scatters multipath
// taps into the corresponding PairBuffer.
//
// Self-paced — no fixed wall-clock cadence. Loops on
// tx_ring.available_read():
//   * if samples available → process up to one batch
//   * if empty             → short fixed sleep (~100 µs), then re-check
//
// Message boundary detection
// --------------------------
// Watches the source's ModemRuntimeState. On TX-entry edge, calls
// Channel::on_message_start on every outgoing channel (which advances the
// PairBuffer write_origin and resets Farrow state). On TX-exit edge (after
// tx_ring is fully drained), calls Channel::on_message_end which exposes
// the multipath tail.
//
// Inter-message gap is measured in wall-clock seconds between successive
// TX messages and converted to receiver-time samples at the source's
// nominal sample rate. The gap is forwarded to Channel::on_message_start
// so PairBuffer places new data at the wall-clock-correct receiver
// position. The first message always uses gap = 0 (no prior TX exit).
class SourceWorker {
public:
    // One outgoing channel: a Channel pipeline plus the destination
    // PairBuffer pointer (owned externally by ChannelEngine).
    //
    // The four "receiver_*" fields below are used by the clock-tracker
    // arrival-alignment path (compile-gated by
    // OPENCRIEST_USE_CLOCK_FILL_TRACKER). They MUST be wired by the
    // ChannelEngine after the receiver's ModemIO has been constructed.
    // In PID mode they're ignored.
    struct Outgoing {
        std::unique_ptr<Channel>  channel;
        PairBuffer*               pair_buffer            = nullptr;
        IFillTracker*             receiver_fill_tracker  = nullptr;
        SPSCRingBuffer<uint16_t>* receiver_rx_ring       = nullptr;
        uint32_t                  receiver_sample_rate   = 500'000;
        double                    propagation_delay_s    = 0.0;
    };

    // Compute the gap (in receiver-time samples) to insert between the
    // prior committed watermark and the new message's first sample so
    // that the modem plays the first sample at
    //   T_tx_start + propagation_delay_s.
    //
    // Clamps a negative result (message arrives "late") to zero and
    // sets *late_out = true if provided. Pure function for testability;
    // used by poll_state_edges under OPENCRIEST_USE_CLOCK_FILL_TRACKER.
    static size_t compute_arrival_aligned_gap(
        std::chrono::steady_clock::time_point T_tx_start,
        double      propagation_delay_s,
        std::chrono::steady_clock::time_point now,
        uint32_t    receiver_fs,
        uint32_t    receiver_fill_samples,
        size_t      pair_buffer_prior_watermark,
        size_t      pair_buffer_read_pos,
        size_t      rx_ring_depth,
        bool*       late_out = nullptr);

    SourceWorker(std::string               source_id,
                 ModemRuntimeState&        runtime,
                 SPSCRingBuffer<uint16_t>& tx_ring,
                 CalibrationData           source_cal,
                 std::vector<Outgoing>     outgoing);

    SourceWorker(const SourceWorker&)            = delete;
    SourceWorker& operator=(const SourceWorker&) = delete;

    // Thread entry; blocks until stop().
    void run();

    // Signal exit; thread-safe.
    void stop();

    bool is_running() const { return running_.load(std::memory_order_relaxed); }

    // Diagnostics
    uint64_t tx_samples_consumed() const { return samples_consumed_; }

    // Wiring (post-construction, called from ChannelEngine after the
    // receiver-side ModemIO has been built). Each call is single-writer
    // and happens before run() is invoked; no synchronization needed.
    void set_source_tx_estimator(TxStartEstimator* est) {
        source_tx_estimator_ = est;
    }
    void set_outgoing_receiver_context(size_t outgoing_idx,
                                        IFillTracker* fill_tracker,
                                        SPSCRingBuffer<uint16_t>* rx_ring) {
        if (outgoing_idx < outgoing_.size()) {
            outgoing_[outgoing_idx].receiver_fill_tracker = fill_tracker;
            outgoing_[outgoing_idx].receiver_rx_ring      = rx_ring;
        }
    }
    size_t num_outgoing() const { return outgoing_.size(); }
    void   set_metrics(Metrics* metrics) { metrics_ = metrics; }

    // Session D observability. Both pointers are optional. When set, the
    // worker records each per-batch process_available() duration into
    // `stats` and emits a MessageEvent at TX-exit (sample_count is the
    // number of source samples consumed during the message). Wired by
    // ChannelEngine after construction, before run().
    void set_processing_time_stats(ProcessingTimeStats* stats) {
        processing_time_stats_ = stats;
    }
    void set_message_event_log(MessageEventLog* log) {
        message_event_log_ = log;
    }
    // Real-time deadline (µs) used to flag a processing tick as
    // "underrun" — defaults to PROCESSING_BLOCK_SIZE / sample_rate.
    // Setting 0 disables underrun counting.
    void set_processing_deadline_us(uint64_t us) {
        processing_deadline_us_ = us;
    }

    // Idle sleep duration when tx_ring is empty.
    static constexpr std::chrono::microseconds kIdleSleep{100};

private:
    // Detect TX entry / TX exit edges; call on_message_start / on_message_end.
    void poll_state_edges();

    // Pull samples, convert ADC→float, run each outgoing channel.
    // Returns samples consumed from tx_ring (zero if nothing to do).
    size_t process_available();

    std::string               source_id_;
    ModemRuntimeState&        runtime_;
    SPSCRingBuffer<uint16_t>& tx_ring_;
    CalibrationData           source_cal_;
    std::vector<Outgoing>     outgoing_;

    std::atomic<bool> running_{false};

    // Last observed source state — for edge detection.
    ModemState last_state_ = ModemState::IDLE;

    // Wall-clock instant of the most recent TX-exit edge. Used on the next
    // TX-entry to compute the inter-message gap (in receiver-time samples
    // at the source's nominal rate) so the new message lands at the right
    // wall-clock position in every receiver's PairBuffer. Empty before the
    // first TX exit.
    std::optional<std::chrono::steady_clock::time_point> last_tx_exit_time_;

    // Source modem's TX-start lower-bound estimator. Wired by
    // ChannelEngine after the source's ModemIO is constructed. Null in
    // PID-tracker mode where it isn't consulted.
    TxStartEstimator* source_tx_estimator_ = nullptr;

    // Process-wide metrics surface; nullptr when not wired (tests).
    Metrics* metrics_ = nullptr;

    // Session D observability. Both null in tests / when disabled.
    ProcessingTimeStats* processing_time_stats_ = nullptr;
    MessageEventLog*     message_event_log_     = nullptr;
    uint64_t             processing_deadline_us_ = 0;

    // Monotonic-per-modem TX sequence id. Incremented at every TX-entry
    // edge; written into the MessageEvent emitted at the matching
    // TX-exit edge. Owned by the source-worker thread, no atomics
    // required (single-threaded with respect to its run loop).
    uint64_t tx_sequence_id_ = 0;
    // Time + sample-count book-keeping for the in-flight message.
    std::chrono::steady_clock::time_point current_msg_start_time_{};
    uint64_t current_msg_start_samples_ = 0;
    bool     current_msg_active_ = false;

    // Per-batch scratch (pre-allocated at construction).
    std::vector<uint16_t> raw_buf_;
    std::vector<float>    float_buf_;

    // Per-outgoing-channel residual: samples Farrow didn't consume on the
    // last call, re-fed at the head of the next batch. Indexed by
    // outgoing_ index.
    std::vector<std::vector<float>> residuals_;

    uint64_t samples_consumed_ = 0;
};

} // namespace openCREST
