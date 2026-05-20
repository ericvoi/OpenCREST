#include "io/modem_io.hpp"
#include "io/pid_fill_tracker.hpp"
#include "io/clock_fill_tracker.hpp"
#include "protocol/packets.hpp"
#include "core/sample_conversion.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

namespace openCREST {

using protocol::ProtocolCodec;
using protocol::StatusPayload;
using protocol::DATA_SAMPLES_PER_PKT;

namespace {

std::unique_ptr<IFillTracker> make_fill_tracker(uint32_t dac_rate,
                                                 uint16_t buffer_capacity,
                                                 uint16_t samples_per_pkt) {
#ifdef OPENCRIEST_USE_CLOCK_FILL_TRACKER
    ClockFillTracker::Config cfg;
    cfg.dac_rate                = dac_rate;
    cfg.buffer_capacity         = buffer_capacity;
    cfg.samples_per_packet      = samples_per_pkt;
    cfg.initial_modem_rate_hint = dac_rate;
    return std::make_unique<ClockFillTracker>(cfg);
#else
    return std::make_unique<PidFillTracker>(dac_rate, buffer_capacity,
                                             samples_per_pkt);
#endif
}

} // namespace

ModemIO::ModemIO(Modem&                    modem,
                  SPSCRingBuffer<uint16_t>& tx_ring,
                  SPSCRingBuffer<uint16_t>& rx_ring,
                  ReceiverMix*              receiver_mix,
                  logging::StreamLogger*    logger,
                  Metrics*                  metrics)
    : modem_(modem)
    , tx_ring_(tx_ring)
    , rx_ring_(rx_ring)
    , receiver_mix_(receiver_mix)
    , logger_(logger)
    , metrics_(metrics)
    , cal_(modem.calibration())
    , tracker_(make_fill_tracker(
          cal_.dac_sampling_rate, MODEM_AUDIO_BUFFER_CAPACITY,
          static_cast<uint16_t>(cal_.samples_per_packet())))
    , tx_start_estimator_(cal_.adc_sampling_rate)
    , samples_per_pkt_(cal_.samples_per_packet())
{}

// ---------------------------------------------------------------------------
// Thread entry point
// ---------------------------------------------------------------------------

void ModemIO::run() {
    running_.store(true, std::memory_order_relaxed);
    size_t iteration = 0;

    spdlog::debug("ModemIO '{}': thread started", modem_.id());

    while (running_.load(std::memory_order_relaxed)) {

        // ---- Control poll ----
        if (iteration % CONTROL_POLL_INTERVAL == 0) {
            poll_control();
        }

        // ---- Data transfer (state-dependent) ----
        const ModemState state =
            modem_.runtime_state().state.load(std::memory_order_relaxed);

        switch (state) {
        case ModemState::TX:
            receive_tx_data();
            break;
        case ModemState::RX:
            send_rx_data();
            break;
        case ModemState::SETTLING:
        case ModemState::IDLE:
            // No data transfer; control is polled above
            break;
        }

        ++iteration;
    }

    spdlog::debug("ModemIO '{}': thread stopped", modem_.id());
}

void ModemIO::stop() {
    running_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void ModemIO::poll_control() {
    const auto result = modem_.transport().recv_control(
        ctrl_buf_, sizeof(ctrl_buf_), CTRL_TIMEOUT_MS);

    if (result.disconnected()) {
        spdlog::error("ModemIO '{}': modem disconnected", modem_.id());
        running_.store(false, std::memory_order_relaxed);
        return;
    }
    // libusb returns OK for zero-length / short reads, and TIMEOUT can come
    // back with a full packet's worth of data when chunks complete just
    // before the timeout fires. Decoding stale ctrl_buf_ on a zero-length
    // OK silently routes last-poll's StatusPayload through handle_status,
    // which can swallow a real upcoming transition by side effect. Gate on
    // a fully-populated 64-byte read regardless of OK vs TIMEOUT.
    if (!result.ok() && !result.timed_out()) return;
    if (result.bytes_transferred != static_cast<int>(CONTROL_PACKET_SIZE)) {
        return;
    }

    StatusPayload status{};
    if (ProtocolCodec::decode_status(ctrl_buf_, status)) {
        handle_status(status);
    }
    // Calibration responses received here are ignored (calibration is
    // done before the I/O thread starts).
}

void ModemIO::handle_status(const StatusPayload& status) {
    const ModemState new_state = static_cast<ModemState>(status.modem_state);
    auto& rt = modem_.runtime_state();

    const ModemState old_state = rt.state.load(std::memory_order_relaxed);

    // Log state transitions (rare events; gives operational visibility
    // for debugging TX/RX/SETTLING cycles).
    if (new_state != old_state) {
        spdlog::info("ModemIO '{}': state {} → {} (fill={}/{}, errs=0x{:02x})",
                     modem_.id(),
                     static_cast<int>(old_state), static_cast<int>(new_state),
                     status.buffer_fill, status.buffer_capacity,
                     status.error_flags);
    }

    // The firmware's resetHil() zeros next_packet_index on every state entry
    // (enterHilRxMode, enterHilTxMode, enterHilTransitionMode, enterHilMode all
    // call it). The MESS subsystem drives HIL state transitions during normal
    // operation, so even an RX→TRANSITIONING→RX cycle resets the firmware's
    // counter. We must mirror that on every transition or host's tx_pkt_id_
    // diverges and ~(2^16 − pipeline_depth) packets are rejected per cycle.
    if (new_state != old_state) {
        // Leaving TX: pull any TX packets still parked in the modem's USB
        // FIFO before publishing the new state. resetHil() in firmware
        // calls tud_vendor_n_write_flush() which *stages* rather than drops
        // pending TX data — without this drain, the last packet of the
        // outgoing session would surface as stale data on the next TX
        // entry and trip sequence-gap detection (observed: gaps at id
        // (msg_len-1) then 0 every cycle).
        if (old_state == ModemState::TX) {
            drain_tx_pipeline();
        }
        codec_.reset_sequence();
        tx_pkt_id_ = 0;
        // Mirror the firmware-side rx_expected_id and host-side
        // tx_pkt_id_ resets in the fill tracker — otherwise the
        // ClockFillTracker's send-timestamp ring would surface stale
        // packet_id collisions across RX sessions. PidFillTracker's
        // reset() is idempotent (BufferPacer::reset clears the same
        // controller state regardless of when it's called).
        tracker_->reset();
        // Entering TX: clear any prior-session observations so the
        // TX-start lower bound is rebuilt from this burst's packets.
        if (new_state == ModemState::TX) {
            tx_start_estimator_.on_tx_entry();
        }
    }

    rt.state.store(new_state, std::memory_order_release);

    // Tracker is only relevant during RX (host→modem injection).
    // During TX the host drains data as fast as possible — no pacing
    // needed. (tracker_->reset on transition is done above.)
    if (new_state == ModemState::RX) {
        if (old_state != ModemState::RX) {
            log_first_rx_send_ = true;
            skip_next_drift_check_ = true;
            if (logger_) logger_->begin_rx_session(modem_.id());
        }
        const auto status_now = std::chrono::steady_clock::now();
        tracker_->on_status(status, status_now);

        // Telemetry: surface the tracker's latest fill estimate, smoothed
        // modem-rate, and accumulated fallback count. PID adapter returns
        // 0 for fallback_anchor_count and estimated_modem_rate by default.
        // Per-modem snapshots are also written for the per-modem
        // diagnostic line (Simulator reads them from a different thread).
        const uint32_t est = tracker_->estimated_fill(status_now);
        const uint32_t rate =
            static_cast<uint32_t>(tracker_->estimated_modem_rate());
        const uint64_t cur_fb = tracker_->fallback_anchor_count();

        last_fill_est_.store(static_cast<int32_t>(est),
                              std::memory_order_relaxed);
        last_modem_rate_.store(rate, std::memory_order_relaxed);
        last_tracker_fallbacks_.store(cur_fb, std::memory_order_relaxed);

        if (metrics_) {
            metrics_->estimated_fill_samples.store(
                static_cast<int32_t>(est), std::memory_order_relaxed);
            metrics_->estimated_modem_rate.store(rate,
                std::memory_order_relaxed);
            if (cur_fb > last_fallback_count_) {
                metrics_->fill_anchor_fallbacks.fetch_add(
                    cur_fb - last_fallback_count_,
                    std::memory_order_relaxed);
                last_fallback_count_ = cur_fb;
            }
        }
    }

    const float fill = (status.buffer_capacity > 0)
        ? static_cast<float>(status.buffer_fill) / status.buffer_capacity
        : 0.5f;
    rt.buffer_fill_fraction.store(fill, std::memory_order_relaxed);

    last_fill_fw_.store(status.buffer_fill, std::memory_order_relaxed);
    if (metrics_) {
        metrics_->fw_buffer_fill.store(status.buffer_fill,
                                        std::memory_order_relaxed);
    }

    rt.error_flags.store(status.error_flags, std::memory_order_relaxed);

    // Counter form of the modem error_flags so a stuck-at-1 condition stays
    // visible in the per-second metrics line (the transition log below goes
    // silent once the bit latches). Bit 0 = RX underrun, bit 1 = TX overrun.
    if (metrics_) {
        if (status.error_flags & 0x01) {
            metrics_->fw_rx_underruns.fetch_add(1, std::memory_order_relaxed);
        }
        if (status.error_flags & 0x02) {
            metrics_->fw_tx_overruns.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // Log error_flags only on transitions (not every packet) to avoid
    // ~400 lines/sec of spam while a bit is stuck set.
    if (status.error_flags != last_error_flags_) {
        spdlog::warn("ModemIO '{}': error_flags 0x{:02x} → 0x{:02x} "
                     "(state={}, fill={}/{})",
                     modem_.id(), last_error_flags_, status.error_flags,
                     static_cast<int>(new_state),
                     status.buffer_fill, status.buffer_capacity);
        last_error_flags_ = status.error_flags;
    }

    // RX id drift = host's next-id-to-send − modem's next-expected-id.
    // Wrap-safe via 16-bit subtraction then sign-extension. A drift *increase*
    // means the firmware silently dropped a packet that the host had sent
    // successfully (rx_expected_id didn't advance for it). That's the
    // diagnostic signal we care about. Steady or decreasing drift is
    // status-arrival-timing noise and not logged.
    //
    // Status reports are emitted by the firmware at fixed packet boundaries
    // (every N accepts) and cross USB asynchronously, so the observed drift
    // naturally wobbles by ±2 packets even when no packet was dropped. Only
    // log when the jump exceeds that ambient jitter band.
    if (new_state == ModemState::RX) {
        constexpr int32_t DRIFT_JITTER_THRESHOLD = 3;
        const int16_t drift = static_cast<int16_t>(tx_pkt_id_ - status.rx_expected_id);
        const int32_t prev  = metrics_ ? metrics_->rx_drift.load(std::memory_order_relaxed) : 0;
        if (metrics_) {
            metrics_->rx_drift.store(drift, std::memory_order_relaxed);
        }
        // On the first status after RX entry, `prev` is stale from before the
        // firmware reset and the USB pipeline is still filling — the measured
        // drift reflects in-flight depth, not drops. Seed the baseline and
        // start comparing from the next report.
        if (skip_next_drift_check_) {
            skip_next_drift_check_ = false;
        } else if (drift - prev > DRIFT_JITTER_THRESHOLD) {
            spdlog::warn("ModemIO '{}': RX drop +{} (drift {} → {}, host_next={}, "
                         "modem_next={}, last_accepted={})",
                         modem_.id(), drift - prev, prev, drift,
                         tx_pkt_id_, status.rx_expected_id,
                         status.fill_reference_id);
        }
    }
}

void ModemIO::receive_tx_data() {
    const auto result = modem_.transport().recv_data(
        data_buf_, sizeof(data_buf_), DATA_TIMEOUT_MS);

    if (result.disconnected()) {
        spdlog::error("ModemIO '{}': disconnect during TX recv", modem_.id());
        running_.store(false, std::memory_order_relaxed);
        return;
    }
    if (!result.ok()) return;

    process_tx_packet(result.bytes_transferred);
}

void ModemIO::process_tx_packet(int bytes_transferred) {
    // libusb returns OK for zero-length and short packets. Decoding those
    // as full data packets reinterprets stale buffer bytes as a packet_id
    // and inflates tx_packets_received.
    if (bytes_transferred != static_cast<int>(DATA_PACKET_SIZE)) {
        if (metrics_) {
            metrics_->short_reads.fetch_add(1, std::memory_order_relaxed);
        }
        return;
    }

    uint16_t pkt_id = 0;
    uint16_t samples[DATA_SAMPLES_PER_PKT];
    size_t   actual = 0;
    ProtocolCodec::decode_data_packet(data_buf_, pkt_id, samples,
                                       DATA_SAMPLES_PER_PKT, actual);

    if (!codec_.check_sequence(pkt_id)) {
        if (metrics_) {
            metrics_->sequence_gaps.fetch_add(1, std::memory_order_relaxed);
        }
        // Rate-limit: log the first few gaps, then only periodically.
        // Logging every packet when every packet is a gap turns into a
        // synchronous-stdout feedback loop that starves the I/O thread.
        const uint64_t n = codec_.gap_count();
        if (n <= 8 || n % 256 == 0) {
            spdlog::warn("ModemIO '{}': TX sequence gap at id {} (total {})",
                         modem_.id(), pkt_id, n);
        }
    }

    const size_t written = tx_ring_.write(samples, actual);
    if (written < actual) {
        spdlog::warn("ModemIO '{}': TX ring full, dropped {} samples",
                     modem_.id(), actual - written);
    }

    // Feed the TX-start lower-bound estimator. The host timestamp is
    // captured here (rather than in receive_tx_data) so the on-exit
    // drain path also contributes. Drain-path packets pre-date the
    // state transition publish, so their lower bound is still valid
    // for the *just-ended* TX session — but we don't use it after
    // on_tx_entry resets state on the next *→TX. Net: drain-path
    // observations are recorded but discarded, which is harmless.
    tx_start_estimator_.on_tx_packet(static_cast<uint16_t>(actual),
                                      std::chrono::steady_clock::now());

    if (metrics_) {
        metrics_->tx_packets_received.fetch_add(1, std::memory_order_relaxed);
    }

    if (logger_) {
        logger_->log_tx(modem_.id(), samples, actual);
    }
}

void ModemIO::drain_tx_pipeline() {
    // Pull packets from the IN endpoint until the modem stops responding
    // within TX_DRAIN_TIMEOUT_MS or we hit the iteration cap. Each packet
    // is fed through the normal TX-side path (process_tx_packet) so its
    // samples reach tx_ring_ exactly as if it had arrived during regular
    // TX-state operation — the message tail is preserved, not discarded.
    for (int i = 0; i < TX_DRAIN_MAX_ITERATIONS; ++i) {
        const auto result = modem_.transport().recv_data(
            data_buf_, sizeof(data_buf_), TX_DRAIN_TIMEOUT_MS);

        if (result.disconnected()) {
            spdlog::error("ModemIO '{}': disconnect during TX drain",
                          modem_.id());
            running_.store(false, std::memory_order_relaxed);
            return;
        }
        // Timeout / no data parked → pipeline empty; we're done.
        if (!result.ok()) break;
        // Short read (typically a USB ZLP) signals the device has no more
        // data ready. Count it for parity with the steady-state path and
        // stop draining.
        if (result.bytes_transferred != static_cast<int>(DATA_PACKET_SIZE)) {
            if (metrics_) {
                metrics_->short_reads.fetch_add(1, std::memory_order_relaxed);
            }
            break;
        }
        process_tx_packet(result.bytes_transferred);
    }
}

void ModemIO::print_metrics_line(double elapsed_s) {
    const uint64_t cur_sent =
        rx_packets_sent_.load(std::memory_order_relaxed);
    const double rx_pps = (elapsed_s > 0.0)
        ? static_cast<double>(cur_sent - prev_rx_packets_sent_) / elapsed_s
        : 0.0;
    prev_rx_packets_sent_ = cur_sent;

    std::printf("[modem %s] RX %.0f pkt/s  fill_est %d  fill_fw %u  "
                "rate %u  fb %llu  skipped %llu\n",
                modem_.id().c_str(),
                rx_pps,
                last_fill_est_.load(std::memory_order_relaxed),
                last_fill_fw_.load(std::memory_order_relaxed),
                last_modem_rate_.load(std::memory_order_relaxed),
                static_cast<unsigned long long>(
                    last_tracker_fallbacks_.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    rx_send_skipped_.load(std::memory_order_relaxed)));
    std::fflush(stdout);
}

void ModemIO::send_rx_data() {
    // Top up rx_ring from the receiver mixer on demand. One packet at a
    // time keeps the ring close to empty in steady state and aligns the
    // PairBuffer drain rate with the modem's DAC consumption rate (one
    // pull per packet sent).
    if (receiver_mix_ && rx_ring_.available_read() < samples_per_pkt_) {
        receiver_mix_->pull(rx_ring_, samples_per_pkt_);
    }
    if (rx_ring_.available_read() < samples_per_pkt_) {
        // Source-side hasn't produced enough samples for a full packet.
        // ReceiverMix::pull zero-pads internally, so this should only fire
        // when the upstream PairBuffer / SourceWorker is genuinely behind —
        // a useful signal for diagnosing modem-side underrun cycles.
        rx_send_skipped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!tracker_->should_send(now)) {
        // Sleep until the next scheduled send. Cap the wait so control polling
        // (every CONTROL_POLL_INTERVAL iterations) stays responsive.
        const auto wait = tracker_->next_send_time() - now;
        const auto cap  = std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(std::chrono::microseconds(500));
        if (wait > std::chrono::steady_clock::duration::zero()) {
            std::this_thread::sleep_for(std::min(wait, cap));
        }
        return;
    }

    uint16_t samples[DATA_SAMPLES_PER_PKT] = {};
    const size_t n = rx_ring_.read(samples, samples_per_pkt_);

    if (log_first_rx_send_) {
        log_first_rx_send_ = false;
        spdlog::info("ModemIO '{}': first RX send at tx_pkt_id_={}",
                     modem_.id(), tx_pkt_id_);
    }

    // Encode with the current id and only advance after the send succeeds —
    // otherwise a failed send silently desyncs the host's counter from the
    // modem's expected_id (observed: ~1 drift per ~5-6k packets sent).
    const uint16_t this_pkt_id = tx_pkt_id_;
    ProtocolCodec::encode_data_packet(data_buf_, this_pkt_id, samples, n);

    const auto result = modem_.transport().send_data(
        data_buf_, sizeof(data_buf_), DATA_TIMEOUT_MS);

    if (result.disconnected()) {
        spdlog::error("ModemIO '{}': disconnect during RX send", modem_.id());
        running_.store(false, std::memory_order_relaxed);
        return;
    }
    if (!result.ok()) {
        spdlog::warn("ModemIO '{}': RX send failed (status {}) at id {}",
                     modem_.id(), static_cast<int>(result.status), this_pkt_id);
        return;
    }

    ++tx_pkt_id_;
    // Capture host-clock just after USB completion so the
    // (packet_id, send_timestamp) pair stored by clock-extrapolation
    // trackers anchors to firmware enqueue time as tightly as possible.
    // PID adapter ignores the timestamp.
    const auto sent_now = std::chrono::steady_clock::now();
    tracker_->on_packet_sent(this_pkt_id,
                             static_cast<uint16_t>(n), sent_now);

    rx_packets_sent_.fetch_add(1, std::memory_order_relaxed);
    if (metrics_) {
        metrics_->rx_packets_sent.fetch_add(1, std::memory_order_relaxed);
    }

    if (logger_) {
        logger_->log_rx(modem_.id(), samples, n);
    }
}

} // namespace openCREST
