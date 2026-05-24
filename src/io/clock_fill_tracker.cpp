#include "io/clock_fill_tracker.hpp"
#include <algorithm>
#include <chrono>

namespace openCREST {

ClockFillTracker::ClockFillTracker(const Config& cfg)
    : cfg_(cfg)
    , target_fill_samples_(
          static_cast<uint32_t>(cfg.target_fill_frac * cfg.buffer_capacity))
    , hyst_low_samples_(
          static_cast<uint32_t>(cfg.hyst_low_frac * cfg.buffer_capacity))
    , hyst_high_samples_(
          static_cast<uint32_t>(cfg.hyst_high_frac * cfg.buffer_capacity))
    , nominal_rate_pkts_per_sec_(
          static_cast<double>(cfg.dac_rate) /
          static_cast<double>(cfg.samples_per_packet))
    , modem_rate_(cfg.initial_modem_rate_hint != 0
                      ? static_cast<double>(cfg.initial_modem_rate_hint)
                      : static_cast<double>(cfg.dac_rate))
{}

void ClockFillTracker::on_packet_sent(uint16_t packet_id,
                                       uint16_t samples,
                                       time_point now) {
    cumulative_samples_sent_ += samples;
    auto& slot = ring_[next_ring_slot_ % RING_CAPACITY];
    slot.packet_id        = packet_id;
    slot.ts               = now;
    slot.cumulative_after = cumulative_samples_sent_;
    slot.valid            = true;
    ++next_ring_slot_;
}

void ClockFillTracker::on_status(const protocol::StatusPayload& status,
                                  time_point now) {
    const Entry* entry = find_entry(status.fill_reference_id);

    // Snapshot previous anchor for rate inference.
    const bool       prev_has         = has_anchor_;
    const int64_t    prev_fill        = anchor_fill_samples_;
    const time_point prev_t           = anchor_time_;
    const uint16_t   prev_rx_expected = prev_rx_expected_id_;
    const bool       prev_rx_valid    = prev_rx_expected_id_valid_;

    if (entry) {
        anchor_fill_samples_       = static_cast<int64_t>(status.buffer_fill);
        anchor_time_               = entry->ts;
        anchor_cumulative_samples_ = entry->cumulative_after;
        last_anchor_was_fallback_  = false;
    } else {
        // Fallback when fill_reference_id isn't in the ring: anchor at
        // status-arrival time. Less accurate but keeps the tracker
        // functional; the following rate inference is skipped because
        // anchor times mix host-send and status-arrival epochs.
        anchor_fill_samples_       = static_cast<int64_t>(status.buffer_fill);
        anchor_time_               = now;
        anchor_cumulative_samples_ = cumulative_samples_sent_;
        last_anchor_was_fallback_  = true;
        ++fallback_anchor_count_;
        skip_next_rate_inference_  = true;
    }
    has_anchor_ = true;

    // EWMA rate inference between consecutive anchors. Counts modem-
    // accepted packets (delta of rx_expected_id × samples_per_pkt), not
    // host-sent samples — otherwise dropped packets would bias the
    // observed rate upward into a self-reinforcing burst loop.
    if (prev_has && prev_rx_valid && !skip_next_rate_inference_) {
        const auto dt = anchor_time_ - prev_t;
        const double dt_seconds =
            std::chrono::duration<double>(dt).count();
        if (dt_seconds > 0) {
            // 16-bit subtract handles the modem's rx_expected_id wrap.
            const int16_t accepted_pkts = static_cast<int16_t>(
                status.rx_expected_id - prev_rx_expected);
            if (accepted_pkts > 0) {
                const int64_t accepted_samples =
                    static_cast<int64_t>(accepted_pkts) *
                    static_cast<int64_t>(cfg_.samples_per_packet);
                const int64_t fill_delta =
                    anchor_fill_samples_ - prev_fill;
                // consumed = accepted − fill_change.
                const int64_t consumed = accepted_samples - fill_delta;
                if (consumed > 0) {
                    const double observed_rate =
                        static_cast<double>(consumed) / dt_seconds;
                    // Reject observations outside [0.5×, 2×] nominal as
                    // status anomalies / stale anchors.
                    const double nominal =
                        static_cast<double>(cfg_.dac_rate);
                    if (observed_rate > nominal * 0.5 &&
                        observed_rate < nominal * 2.0) {
                        modem_rate_ =
                            (1.0 - cfg_.rate_ewma_alpha) * modem_rate_
                          + cfg_.rate_ewma_alpha * observed_rate;
                    }
                }
            }
        }
    }
    prev_rx_expected_id_       = status.rx_expected_id;
    prev_rx_expected_id_valid_ = true;
    skip_next_rate_inference_  = false;
}

const ClockFillTracker::Entry*
ClockFillTracker::find_entry(uint16_t packet_id) const {
    for (const auto& e : ring_) {
        if (e.valid && e.packet_id == packet_id) return &e;
    }
    return nullptr;
}

int64_t ClockFillTracker::extrapolated_fill(time_point now) const {
    if (!has_anchor_) {
        return static_cast<int64_t>(target_fill_samples_);
    }
    const int64_t sent_since_anchor = static_cast<int64_t>(
        cumulative_samples_sent_) -
        static_cast<int64_t>(anchor_cumulative_samples_);
    const double elapsed_s =
        std::chrono::duration<double>(now - anchor_time_).count();
    const int64_t consumed = static_cast<int64_t>(
        elapsed_s * modem_rate_ + 0.5);
    int64_t fill = anchor_fill_samples_ + sent_since_anchor - consumed;
    fill = std::max<int64_t>(fill, 0);
    fill = std::min<int64_t>(fill,
                              static_cast<int64_t>(cfg_.buffer_capacity));
    return fill;
}

uint32_t ClockFillTracker::estimated_fill(time_point now) const {
    return static_cast<uint32_t>(extrapolated_fill(now));
}

bool ClockFillTracker::should_send(time_point now) {
    if (!schedule_initialized_) {
        next_send_time_       = now;
        last_send_time_       = now;
        schedule_initialized_ = true;
    }

    const int64_t fill = extrapolated_fill(now);

    // Two-signal hold decision: default to extrapolation (smooth, sub-ms
    // resolution); switch to the anchor when they disagree by more than
    // half the target fill — that's the fingerprint of modem_rate_
    // having drifted from the actual drain rate. Anchor staleness is
    // deliberately NOT used to switch back to extrap during hold: a
    // stale-high anchor means the modem is rejecting our packets, and
    // trusting extrap there produces an overflow loop. The keepalive
    // trickle below is what breaks the deadlock — one packet every
    // KEEPALIVE_INTERVAL eventually gets accepted, refreshing the
    // anchor with ground truth.
    const int64_t disagreement_threshold =
        static_cast<int64_t>(target_fill_samples_ / 2);
    const int64_t disagreement = has_anchor_
        ? std::abs(fill - anchor_fill_samples_)
        : 0;
    const bool trust_anchor =
        has_anchor_ && disagreement > disagreement_threshold;
    const int64_t effective_fill =
        trust_anchor ? anchor_fill_samples_ : fill;

    if (!holding_ && effective_fill > static_cast<int64_t>(
            target_fill_samples_ + hyst_high_samples_)) {
        holding_ = true;
    } else if (holding_ && effective_fill < static_cast<int64_t>(target_fill_samples_)) {
        holding_ = false;
    }

    if (holding_) {
        // Keepalive trickle (see hold-decision comment above). 50 ms ×
        // 256 samples ≈ 5 k samples/s — well below drain (~500 k), so
        // the trickle can't keep a full buffer full, only prevent the
        // host from going silent.
        constexpr auto KEEPALIVE_INTERVAL = std::chrono::milliseconds(50);
        if (now - last_send_time_ < KEEPALIVE_INTERVAL) return false;
        last_send_time_ = now;
        return true;
    }

    if (now < next_send_time_) return false;

    // Choose period based on how far below target we are:
    //   < target - hyst_low_samples_ → burst rate
    //   otherwise                    → nominal rate
    const bool deeply_low = fill < static_cast<int64_t>(target_fill_samples_) -
                                    static_cast<int64_t>(hyst_low_samples_);
    const double rate = deeply_low
        ? nominal_rate_pkts_per_sec_ * cfg_.burst_cap_multiplier
        : nominal_rate_pkts_per_sec_;
    const auto period = std::chrono::duration_cast<time_point::duration>(
        std::chrono::duration<double>(1.0 / rate));
    next_send_time_ += period;
    // Bounded catch-up after an I/O stall.
    constexpr int MAX_CATCHUP_SLOTS = 3;
    if (now - next_send_time_ > period * MAX_CATCHUP_SLOTS) {
        next_send_time_ = now + period;
    }
    last_send_time_ = now;
    return true;
}

void ClockFillTracker::reset() {
    // Clear anchor + ring + cumulative + schedule. The smoothed
    // modem_rate_ is preserved across resets (hardware constant).
    has_anchor_                = false;
    anchor_fill_samples_       = 0;
    anchor_time_               = {};
    anchor_cumulative_samples_ = 0;
    last_anchor_was_fallback_  = false;
    skip_next_rate_inference_  = true;

    for (auto& e : ring_) e.valid = false;
    next_ring_slot_ = 0;

    cumulative_samples_sent_ = 0;

    // Mirror the firmware's per-state-transition rx_expected_id reset.
    prev_rx_expected_id_       = 0;
    prev_rx_expected_id_valid_ = false;

    next_send_time_       = {};
    last_send_time_       = {};
    schedule_initialized_ = false;
    holding_              = false;
}

} // namespace openCREST
