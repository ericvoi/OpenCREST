#include "channel/source_worker.hpp"
#include "core/constants.hpp"
#include "core/sample_conversion.hpp"

#include <algorithm>
#include <thread>

namespace openCREST {

namespace {

// steady_clock → uint64_t nanoseconds since epoch. Used as the event-log
// timestamp; consumers (Python harness) treat it as a monotonic value
// rather than wall-clock.
uint64_t steady_ns(std::chrono::steady_clock::time_point t) {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            t.time_since_epoch()).count());
}

} // namespace

size_t SourceWorker::compute_arrival_aligned_gap(
    std::chrono::steady_clock::time_point T_tx_start,
    double      propagation_delay_s,
    std::chrono::steady_clock::time_point now,
    uint32_t    receiver_fs,
    uint32_t    receiver_fill_samples,
    size_t      pair_buffer_prior_watermark,
    size_t      pair_buffer_read_pos,
    size_t      rx_ring_depth,
    bool*       late_out) {
    const auto target = T_tx_start +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(propagation_delay_s));
    const double dt_s =
        std::chrono::duration<double>(target - now).count();
    const int64_t in_flight = static_cast<int64_t>(
        pair_buffer_prior_watermark - pair_buffer_read_pos);
    const int64_t gap = static_cast<int64_t>(
            dt_s * static_cast<double>(receiver_fs))
        - in_flight
        - static_cast<int64_t>(rx_ring_depth)
        - static_cast<int64_t>(receiver_fill_samples);
    if (gap < 0) {
        if (late_out) *late_out = true;
        return 0;
    }
    if (late_out) *late_out = false;
    return static_cast<size_t>(gap);
}

SourceWorker::SourceWorker(std::string               source_id,
                           ModemRuntimeState&        runtime,
                           SPSCRingBuffer<uint16_t>& tx_ring,
                           CalibrationData           source_cal,
                           std::vector<Outgoing>     outgoing)
    : source_id_(std::move(source_id))
    , runtime_(runtime)
    , tx_ring_(tx_ring)
    , source_cal_(source_cal)
    , outgoing_(std::move(outgoing))
{
    // Determine the largest input batch any outgoing channel needs so the
    // raw + float scratch buffers cover them all.
    size_t max_batch = PROCESSING_BLOCK_SIZE;
    for (const auto& og : outgoing_) {
        max_batch = std::max(max_batch, og.channel->input_needed_for_batch());
    }
    raw_buf_.assign  (max_batch, 0);
    float_buf_.assign(max_batch, 0.0f);

    residuals_.resize(outgoing_.size());
    // Reserve plenty so push_back/insert don't reallocate on the hot path.
    for (auto& r : residuals_) r.reserve(max_batch * 2);
}

void SourceWorker::run() {
    running_.store(true, std::memory_order_release);
    // Initialize last_state_ to IDLE (a non-TX sentinel) rather than the
    // live state. If the runtime state is already TX when the worker
    // starts, the first poll_state_edges will treat that as an IDLE→TX
    // edge and correctly fire on_message_start. Capturing the live state
    // here would race with anyone who sets state to TX before the worker
    // thread is scheduled (the test thread typically loses that race).
    last_state_ = ModemState::IDLE;

    while (running_.load(std::memory_order_relaxed)) {
        poll_state_edges();

        // Only drain tx_ring while in TX. In other states the ring is not
        // expected to grow, but we still loop fast enough to catch the next
        // edge promptly.
        const ModemState state = runtime_.state.load(std::memory_order_acquire);
        size_t processed = 0;
        if (state == ModemState::TX) {
            processed = process_available();
        }

        if (processed == 0) {
            std::this_thread::sleep_for(kIdleSleep);
        }
    }

    // Cleanup pass: handle any pending state edge that arrived between the
    // last loop iteration and stop(). Ensures on_message_end fires when
    // the state transitioned out of TX just before shutdown.
    poll_state_edges();

    // If we're still nominally in TX at shutdown (stop() called mid-message),
    // synthesize a TX-exit so the multipath tail is flushed and the receiver
    // doesn't see a half-message.
    if (last_state_ == ModemState::TX) {
        while (tx_ring_.available_read() > 0) {
            if (process_available() == 0) break;
        }
        for (size_t i = 0; i < outgoing_.size(); ++i) {
            auto& og  = outgoing_[i];
            auto& res = residuals_[i];
            if (!res.empty()) {
                const size_t consumed = og.channel->process(
                    res.data(), res.size(), *og.pair_buffer);
                if (consumed >= res.size()) res.clear();
                else res.erase(res.begin(),
                                res.begin() + static_cast<ptrdiff_t>(consumed));
            }
            og.channel->on_message_end(*og.pair_buffer);
        }
        last_state_ = ModemState::IDLE;
    }
}

void SourceWorker::stop() {
    running_.store(false, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// State edges
// ---------------------------------------------------------------------------

void SourceWorker::poll_state_edges() {
    const ModemState now = runtime_.state.load(std::memory_order_acquire);
    if (now == last_state_) return;

    const bool was_tx = (last_state_ == ModemState::TX);
    const bool is_tx  = (now         == ModemState::TX);

    if (!was_tx && is_tx) {
        // TX entry: per outgoing channel, begin a new message.
#ifdef OPENCRIEST_USE_CLOCK_FILL_TRACKER
        // Clock-tracker arrival-alignment path. Place write_origin so
        // the message's first sample plays out of the receiver modem at
        // host-time T_tx_start_source + propagation_delay_s.
        //
        // Lower bound on TX-start: tightest observation from the
        // source's TxStartEstimator, or steady_clock::now() if no
        // packet has arrived yet (per plan: cold-start error is
        // bounded by the per-packet USB period, ~510 µs).
        const auto now = std::chrono::steady_clock::now();
        const auto T_tx_start =
            (source_tx_estimator_ && source_tx_estimator_->has_observation())
                ? *source_tx_estimator_->earliest_start_time()
                : now;

        for (auto& og : outgoing_) {
            size_t gap_samples       = 0;
            bool   arrival_alignment = false;
            bool   late              = false;
            if (og.receiver_fill_tracker && og.receiver_rx_ring) {
                const uint32_t fb_est =
                    og.receiver_fill_tracker->estimated_fill(now);
                const uint32_t rx_ring_depth = static_cast<uint32_t>(
                    og.receiver_rx_ring->available_read());
                const size_t prior_wm = og.pair_buffer->write_watermark();
                const size_t read_pos = og.pair_buffer->read_pos();
                gap_samples = compute_arrival_aligned_gap(
                    T_tx_start,
                    og.propagation_delay_s,
                    now,
                    og.receiver_sample_rate,
                    fb_est,
                    prior_wm,
                    read_pos,
                    rx_ring_depth,
                    &late);
                if (late && metrics_) {
                    metrics_->late_messages.fetch_add(
                        1, std::memory_order_relaxed);
                }
                arrival_alignment = true;
            }
            // absolute_first_origin tracks arrival_alignment: when the
            // fill-tracker is present, the gap incorporates propagation
            // delay AND pipeline-depth corrections — auto-applying
            // base_delay on the first message would double-shift and
            // re-introduce the ~25 m one-shot ranging bias on the first
            // ranging after simulator restart.
            //
            // Without a fill-tracker (unit tests, minimal harnesses) the
            // gap is 0 and the legacy PID-mode auto-base-delay path is the
            // right behavior — PairBuffer encodes propagation delay
            // structurally rather than via gap math.
            og.channel->on_message_start(*og.pair_buffer, gap_samples,
                                          arrival_alignment);
        }
#else
        // PID-tracker path: gap is forced to 0; PairBuffer auto-applies
        // base_delay on the first message (legacy behavior). See
        // /home/ern/.claude/plans/i-am-going-through-functional-comet.md
        // for why PID's fill estimate isn't precise enough for sample-
        // accurate alignment.
        const size_t gap_samples = 0;
        for (auto& og : outgoing_) {
            og.channel->on_message_start(*og.pair_buffer, gap_samples);
        }
#endif
        // Discard any residual from a prior message — Farrow has been reset.
        for (auto& r : residuals_) r.clear();

        // Session D — open a new in-flight message record. The matching
        // record() fires at TX-exit below.
        if (message_event_log_) {
            current_msg_start_time_     = std::chrono::steady_clock::now();
            current_msg_start_samples_  = samples_consumed_;
            current_msg_active_         = true;
        }
    } else if (was_tx && !is_tx) {
        // TX exit: drain any tx_ring tail still queued, then flush each
        // channel so the multipath tail becomes visible to the receiver.
        // We loop until the ring is empty so no source samples are lost.
        while (tx_ring_.available_read() > 0) {
            if (process_available() == 0) break;
        }
        for (size_t i = 0; i < outgoing_.size(); ++i) {
            // Flush any residual samples through (one final partial batch).
            auto& og  = outgoing_[i];
            auto& res = residuals_[i];
            if (!res.empty()) {
                const size_t consumed = og.channel->process(
                    res.data(), res.size(), *og.pair_buffer);
                if (consumed >= res.size()) res.clear();
                else res.erase(res.begin(),
                                res.begin() + static_cast<ptrdiff_t>(consumed));
            }
            og.channel->on_message_end(*og.pair_buffer);
        }
        // Stamp the wall-clock exit time so the next TX-entry can compute
        // the inter-message gap.
        last_tx_exit_time_ = std::chrono::steady_clock::now();

        // Session D — close the in-flight message record. Suppressed when
        // no opening edge was seen (e.g. mid-run wiring), so the log
        // never emits half-records.
        if (message_event_log_ && current_msg_active_) {
            MessageEvent ev;
            ev.modem_id     = source_id_;
            ev.direction    = MessageEvent::Direction::Tx;
            ev.start_ns     = steady_ns(current_msg_start_time_);
            ev.end_ns       = steady_ns(*last_tx_exit_time_);
            ev.sample_count = samples_consumed_ - current_msg_start_samples_;
            ev.sequence_id  = tx_sequence_id_++;
            message_event_log_->record(ev);
            current_msg_active_ = false;
        }
    }

    last_state_ = now;
}

// ---------------------------------------------------------------------------
// Per-batch processing
// ---------------------------------------------------------------------------

size_t SourceWorker::process_available() {
    const size_t avail = tx_ring_.available_read();
    if (avail == 0) return 0;

    // RAII timer for Session D processing-time histogram. Constructed
    // only when the stats sink is wired so the disabled path costs
    // nothing beyond a null-pointer check.
    struct TimerGuard {
        ProcessingTimeStats* stats;
        uint64_t             deadline_us;
        std::chrono::steady_clock::time_point t0;
        ~TimerGuard() {
            if (!stats) return;
            const auto dt =
                std::chrono::steady_clock::now() - t0;
            const uint64_t us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(dt)
                    .count());
            stats->record(us, deadline_us);
        }
    } _timer{processing_time_stats_, processing_deadline_us_,
              std::chrono::steady_clock::now()};

    // Read a batch; bounded by raw_buf_ capacity and a per-tick batch size to
    // keep latency predictable.
    const size_t batch = std::min(avail, raw_buf_.size());
    const size_t got   = tx_ring_.read(raw_buf_.data(), batch);
    if (got == 0) return 0;

    // ADC → float (one conversion shared across all outgoing channels).
    const uint8_t adc_bits = source_cal_.adc_bits;
    for (size_t i = 0; i < got; ++i) {
        float_buf_[i] = adc_to_float(raw_buf_[i], adc_bits);
    }

    samples_consumed_ += got;

    // Hand the same source-data block to every outgoing channel. Each
    // channel maintains its own residual buffer (Farrow leftovers).
    for (size_t i = 0; i < outgoing_.size(); ++i) {
        auto& og  = outgoing_[i];
        auto& res = residuals_[i];

        // Build the input for this channel: prior residual followed by the
        // new samples. To avoid an extra allocation, we copy the new samples
        // onto the end of `res`, process from `res`, then trim consumed.
        const size_t prior_residual = res.size();
        res.insert(res.end(), float_buf_.data(), float_buf_.data() + got);

        const size_t consumed = og.channel->process(
            res.data(), res.size(), *og.pair_buffer);

        if (consumed > 0) {
            // Drop consumed samples from the front of `res`.
            res.erase(res.begin(),
                       res.begin() + static_cast<ptrdiff_t>(consumed));
        }
        // residual_left = res.size() (unconsumed Farrow samples for next call)
        (void)prior_residual;
    }

    return got;
}

} // namespace openCREST
