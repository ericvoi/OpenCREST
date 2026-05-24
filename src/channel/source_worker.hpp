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
// Drains the source's tx_ring, converts ADC -> float, and feeds each
// outgoing Channel which scatters multipath taps into its PairBuffer.
//
// Self-paced (no fixed cadence): polls tx_ring.available_read(),
// processes up to one batch when non-empty, sleeps kIdleSleep when empty.
//
// Watches the source's ModemRuntimeState. On TX-entry calls
// Channel::on_message_start on every outgoing channel (advances
// write_origin, resets Farrow). On TX-exit (after tx_ring drains) calls
// Channel::on_message_end (exposes multipath tail).
class SourceWorker {
public:
    // One outgoing channel: pipeline + destination PairBuffer (owned by
    // ChannelEngine).
    //
    // receiver_fill_tracker and receiver_rx_ring feed the clock-tracker
    // arrival-alignment path (compile-gated by
    // OPENCREST_USE_CLOCK_FILL_TRACKER). Wired post-construction by
    // ChannelEngine once the receiver's ModemIO exists; ignored in PID mode.
    struct Outgoing {
        std::unique_ptr<Channel>  channel;
        PairBuffer*               pair_buffer            = nullptr;
        IFillTracker*             receiver_fill_tracker  = nullptr;
        SPSCRingBuffer<uint16_t>* receiver_rx_ring       = nullptr;
        uint32_t                  receiver_sample_rate   = 500'000;
        double                    propagation_delay_s    = 0.0;
    };

    // Gap in receiver-time samples between the prior watermark and the
    // new message's first sample, such that the receiver plays the first
    // sample at T_tx_start + propagation_delay_s. Clamps negative results
    // (late arrival) to zero and sets *late_out=true. Pure function.
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

    uint64_t tx_samples_consumed() const { return samples_consumed_; }

    // Wiring setters. Single-writer; called before run().
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

    // Optional observability sinks. When set, the worker records each
    // process_available() duration and emits a MessageEvent at TX-exit.
    void set_processing_time_stats(ProcessingTimeStats* stats) {
        processing_time_stats_ = stats;
    }
    void set_message_event_log(MessageEventLog* log) {
        message_event_log_ = log;
    }
    // Real-time deadline (us) used to flag a processing tick as underrun.
    // 0 disables underrun counting.
    void set_processing_deadline_us(uint64_t us) {
        processing_deadline_us_ = us;
    }

    // Idle sleep duration when tx_ring is empty.
    static constexpr std::chrono::microseconds kIdleSleep{100};

private:
    // Detect TX-entry / TX-exit edges and fire the lifecycle callbacks.
    void poll_state_edges();

    // Pull samples, convert ADC->float, run each outgoing channel.
    // Returns samples consumed from tx_ring (0 if nothing to do).
    size_t process_available();

    std::string               source_id_;
    ModemRuntimeState&        runtime_;
    SPSCRingBuffer<uint16_t>& tx_ring_;
    CalibrationData           source_cal_;
    std::vector<Outgoing>     outgoing_;

    std::atomic<bool> running_{false};

    // Last observed source state, for edge detection.
    ModemState last_state_ = ModemState::IDLE;

    // Wall-clock instant of the most recent TX-exit. Used to compute the
    // next TX-entry's inter-message gap. Empty before the first TX exit.
    std::optional<std::chrono::steady_clock::time_point> last_tx_exit_time_;

    // Source's TX-start lower-bound estimator. Null when not wired.
    TxStartEstimator* source_tx_estimator_ = nullptr;

    // Optional. Null when not wired.
    Metrics* metrics_ = nullptr;
    ProcessingTimeStats* processing_time_stats_ = nullptr;
    MessageEventLog*     message_event_log_     = nullptr;
    uint64_t             processing_deadline_us_ = 0;

    // Monotonic-per-modem TX sequence id. Incremented at each TX-entry,
    // written into the MessageEvent at the matching TX-exit. No atomics
    // (worker-thread private).
    uint64_t tx_sequence_id_ = 0;
    // In-flight message book-keeping.
    std::chrono::steady_clock::time_point current_msg_start_time_{};
    uint64_t current_msg_start_samples_ = 0;
    bool     current_msg_active_ = false;

    // Per-batch scratch (pre-allocated).
    std::vector<uint16_t> raw_buf_;
    std::vector<float>    float_buf_;

    // Per-outgoing-channel residual: samples Farrow didn't consume on the
    // last call, re-fed at the head of the next batch.
    std::vector<std::vector<float>> residuals_;

    uint64_t samples_consumed_ = 0;
};

} // namespace openCREST
