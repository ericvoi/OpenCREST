#include "channel/source_worker.hpp"
#include "core/constants.hpp"
#include "core/sample_conversion.hpp"

#include <algorithm>
#include <thread>

namespace openCREST {

namespace {

// steady_clock -> nanoseconds since epoch. Used as the event-log timestamp;
// consumers treat it as monotonic, not wall-clock.
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
    // Size scratch to the largest input batch any outgoing channel needs.
    size_t max_batch = PROCESSING_BLOCK_SIZE;
    for (const auto& og : outgoing_) {
        max_batch = std::max(max_batch, og.channel->input_needed_for_batch());
    }
    raw_buf_.assign  (max_batch, 0);
    float_buf_.assign(max_batch, 0.0f);

    residuals_.resize(outgoing_.size());
    // Reserve so insert() doesn't reallocate on the hot path.
    for (auto& r : residuals_) r.reserve(max_batch * 2);
}

void SourceWorker::run() {
    running_.store(true, std::memory_order_release);
    // Seed last_state_ as IDLE (not the live state) so a worker that
    // starts while runtime is already TX fires on_message_start on the
    // first poll. Reading the live state here would race with whoever set
    // it to TX before this thread was scheduled.
    last_state_ = ModemState::IDLE;

    while (running_.load(std::memory_order_relaxed)) {
        poll_state_edges();

        // Only drain tx_ring while in TX; the loop still spins fast enough
        // to catch the next edge promptly.
        const ModemState state = runtime_.state.load(std::memory_order_acquire);
        size_t processed = 0;
        if (state == ModemState::TX) {
            processed = process_available();
        }

        if (processed == 0) {
            std::this_thread::sleep_for(kIdleSleep);
        }
    }

    // Catch any edge that arrived between the last iteration and stop().
    poll_state_edges();

    // If still nominally in TX at shutdown, synthesize a TX-exit so the
    // multipath tail is flushed and the receiver doesn't see a half-message.
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

void SourceWorker::poll_state_edges() {
    const ModemState now = runtime_.state.load(std::memory_order_acquire);
    if (now == last_state_) return;

    const bool was_tx = (last_state_ == ModemState::TX);
    const bool is_tx  = (now         == ModemState::TX);

    if (!was_tx && is_tx) {
        // TX entry: begin a new message on every outgoing channel.
#ifdef OPENCREST_USE_CLOCK_FILL_TRACKER
        // Arrival-alignment: place write_origin so the message's first
        // sample plays out at T_tx_start_source + propagation_delay_s.
        // Lower bound on TX-start is the tightest TxStartEstimator
        // observation, or now() if no packet has arrived yet (cold-start
        // error bounded by the per-packet USB period, ~510 us).
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
            // fill-tracker is present the gap already includes propagation
            // delay and pipeline-depth corrections, so PairBuffer must not
            // auto-apply base_delay (would double-shift). Without a
            // fill-tracker the PID-mode structural-delay path is correct.
            og.channel->on_message_start(*og.pair_buffer, gap_samples,
                                          arrival_alignment);
        }
#else
        // PID-tracker path: gap forced to 0; PairBuffer auto-applies
        // base_delay on the first message. PID fill estimate isn't precise
        // enough for sample-accurate alignment.
        const size_t gap_samples = 0;
        for (auto& og : outgoing_) {
            og.channel->on_message_start(*og.pair_buffer, gap_samples);
        }
#endif
        // Discard prior-message residual; Farrow has been reset.
        for (auto& r : residuals_) r.clear();

        // Open the in-flight message record; matching record() fires at TX-exit.
        if (message_event_log_) {
            current_msg_start_time_     = std::chrono::steady_clock::now();
            current_msg_start_samples_  = samples_consumed_;
            current_msg_active_         = true;
        }
    } else if (was_tx && !is_tx) {
        // TX exit: drain the tx_ring tail, then flush each channel so the
        // multipath tail becomes visible. Loop until empty so no samples
        // are lost.
        while (tx_ring_.available_read() > 0) {
            if (process_available() == 0) break;
        }
        for (size_t i = 0; i < outgoing_.size(); ++i) {
            // One final partial batch for any residual.
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
        // Stamp the exit time for the next TX-entry's gap computation.
        last_tx_exit_time_ = std::chrono::steady_clock::now();

        // Close the in-flight message record. Suppressed when no opening
        // edge was seen (e.g. mid-run wiring) so the log never emits
        // half-records.
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

size_t SourceWorker::process_available() {
    const size_t avail = tx_ring_.available_read();
    if (avail == 0) return 0;

    // RAII timer for the processing-time histogram. The disabled path
    // costs only a null-pointer check in the destructor.
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

    // Bounded by raw_buf_ capacity to keep per-tick latency predictable.
    const size_t batch = std::min(avail, raw_buf_.size());
    const size_t got   = tx_ring_.read(raw_buf_.data(), batch);
    if (got == 0) return 0;

    // ADC -> float, shared across all outgoing channels.
    const uint8_t adc_bits = source_cal_.adc_bits;
    for (size_t i = 0; i < got; ++i) {
        float_buf_[i] = adc_to_float(raw_buf_[i], adc_bits);
    }

    samples_consumed_ += got;

    // Feed the same source block to every outgoing channel. Each carries
    // its own Farrow residual.
    for (size_t i = 0; i < outgoing_.size(); ++i) {
        auto& og  = outgoing_[i];
        auto& res = residuals_[i];

        // Append new samples onto `res` so we can pass {residual, new} as
        // one contiguous buffer without an extra allocation; trim consumed
        // afterwards.
        const size_t prior_residual = res.size();
        res.insert(res.end(), float_buf_.data(), float_buf_.data() + got);

        const size_t consumed = og.channel->process(
            res.data(), res.size(), *og.pair_buffer);

        if (consumed > 0) {
            res.erase(res.begin(),
                       res.begin() + static_cast<ptrdiff_t>(consumed));
        }
        (void)prior_residual;
    }

    return got;
}

} // namespace openCREST
