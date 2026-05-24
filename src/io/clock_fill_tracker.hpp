#pragma once
#include "core/fill_tracker.hpp"
#include <array>
#include <cstdint>

namespace openCREST {

// Clock-extrapolation buffer-fill tracker.
//
// Anchors each modem fill measurement to the host-side send timestamp of
// the firmware-identified `fill_reference_id` packet, then extrapolates
// fill at any later time via:
//
//   fill(T) = F_anchor
//           + (cumulative_samples_sent_now − samples_sent_at_anchor)
//           − round((T − T_anchor) × Fs_modem)
//
// Fs_modem is seeded from calibration and refined between anchors via
// EWMA of the observed (consumed_samples / Δt) ratio. The first
// inference after reset() is skipped because pipeline depth makes the
// initial measurement unreliable.
//
// Send-decision logic: bang-bang with asymmetric hysteresis around the
// target fill, capped at burst_cap_multiplier × nominal_rate.
//
// Thread-safety: instance owned by a single ModemIO; no internal
// locking.
class ClockFillTracker final : public IFillTracker {
public:
    struct Config {
        uint32_t dac_rate                 = 500'000;
        uint16_t buffer_capacity          = 16'384;
        uint16_t samples_per_packet       = 255;
        float    target_fill_frac         = 0.5f;
        // Hysteresis fractions of buffer_capacity:
        //   enter "hold" when fill > target + hyst_high_frac
        //   exit  "hold" when fill < target  (asymmetric)
        // Plus a "burst" zone:
        //   send at burst rate when fill < target - hyst_low_frac
        //   send at nominal rate otherwise (inside hyst_low band)
        float    hyst_low_frac            = 0.02f;
        float    hyst_high_frac           = 0.02f;
        float    burst_cap_multiplier     = 1.5f;
        // EWMA coefficient applied to new rate observations:
        //   modem_rate_ ← (1-α) × modem_rate_ + α × observed
        float    rate_ewma_alpha          = 0.2f;
        // Seed for the modem-rate smoother. Defaults to dac_rate.
        uint32_t initial_modem_rate_hint  = 0;  // 0 → use dac_rate
    };

    explicit ClockFillTracker(const Config& cfg);

    // IFillTracker
    void on_status(const protocol::StatusPayload& status,
                   time_point now) override;
    void on_packet_sent(uint16_t packet_id, uint16_t samples,
                        time_point now) override;
    bool should_send(time_point now) override;
    time_point next_send_time() const override { return next_send_time_; }
    uint32_t estimated_fill(time_point now) const override;
    void reset() override;

    // Diagnostics
    uint64_t fallback_anchor_count() const override { return fallback_anchor_count_; }
    double   estimated_modem_rate()  const override { return modem_rate_; }
    bool     last_anchor_was_fallback() const { return last_anchor_was_fallback_; }
    bool     has_anchor() const { return has_anchor_; }

private:
    struct Entry {
        uint16_t   packet_id;
        time_point ts;
        uint64_t   cumulative_after;  // total samples sent through this packet
        bool       valid;
    };

    static constexpr size_t RING_CAPACITY = 64;

    const Entry* find_entry(uint16_t packet_id) const;
    int64_t      extrapolated_fill(time_point now) const;

    Config cfg_;

    // Derived bounds, in samples.
    uint32_t target_fill_samples_;
    uint32_t hyst_low_samples_;
    uint32_t hyst_high_samples_;
    double   nominal_rate_pkts_per_sec_;
    double   modem_rate_;        // smoothed Fs_modem (samples/sec)

    // Anchor state.
    bool       has_anchor_                 = false;
    int64_t    anchor_fill_samples_        = 0;
    time_point anchor_time_{};
    uint64_t   anchor_cumulative_samples_  = 0;
    bool       last_anchor_was_fallback_   = false;
    bool       skip_next_rate_inference_   = true;

    // Send-timestamp ring (linear scan; small enough that hashing
    // wouldn't pay).
    std::array<Entry, RING_CAPACITY> ring_{};
    size_t next_ring_slot_ = 0;

    // Cumulative samples sent during the current RX session.
    uint64_t cumulative_samples_sent_ = 0;

    // Most recent modem-reported rx_expected_id. Deltas drive the rate
    // EWMA — counting modem-accepted packets, not host-sent ones, so
    // dropped packets don't poison the smoothed rate.
    uint16_t prev_rx_expected_id_ = 0;
    bool     prev_rx_expected_id_valid_ = false;

    uint64_t fallback_anchor_count_ = 0;

    // Send schedule.
    time_point next_send_time_{};
    bool       schedule_initialized_ = false;
    bool       holding_              = false;  // hysteresis state

    // Wall-clock of the most recent send. Used to rate-limit the
    // keepalive trickle during hold so the modem keeps emitting status
    // (firmware only emits status on accepted incoming packets).
    time_point last_send_time_{};
};

} // namespace openCREST
