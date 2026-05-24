#pragma once
#include <chrono>
#include <cstdint>
#include "protocol/packets.hpp"

namespace openCREST {

// Host-side estimator for a modem's RX audio-buffer fill level.
//
// Owns the pacing decision (when to send the next host→modem RX data
// packet) and exposes the current extrapolated fill level (used by the
// channel pipeline to align message arrival times).
//
// Two implementations exist:
//   - PidFillTracker    : PID loop on reported fill.
//   - ClockFillTracker  : anchors fill to the host-side send timestamp of
//                         the firmware-named fill_reference_id and
//                         extrapolates from there.
// Selection is compile-time via OPENCREST_USE_CLOCK_FILL_TRACKER.
//
// Thread-safety: each instance is owned by a single ModemIO; all methods
// run on that thread.
class IFillTracker {
public:
    using clock      = std::chrono::steady_clock;
    using time_point = clock::time_point;

    virtual ~IFillTracker() = default;

    // Called for every status packet received from this modem (during RX).
    // `now` is the host time at status-packet arrival.
    virtual void on_status(const protocol::StatusPayload& status,
                           time_point now) = 0;

    // Called after every successful host→modem RX data packet send.
    // `packet_id` is the wire-level packet_id (mirrors the firmware's
    // rx_expected_id counter); `samples` is the audio sample count in that
    // packet (255 in steady state).
    virtual void on_packet_sent(uint16_t packet_id,
                                uint16_t samples,
                                time_point now) = 0;

    // True if a packet should be sent at `now`. Advances the schedule by
    // one period on return-true; callers must honour that.
    virtual bool should_send(time_point now) = 0;

    // Absolute time of the next scheduled send. Callers may sleep until
    // this point to avoid busy-waiting.
    virtual time_point next_send_time() const = 0;

    // Extrapolated current fill level in samples.
    virtual uint32_t estimated_fill(time_point now) const = 0;

    // Clear all state. Called on every modem state transition (mirrors
    // the firmware's resetHil() which zeros next_packet_index on entry).
    virtual void reset() = 0;

    // Cumulative count of status packets whose fill_reference_id was not
    // in the send-timestamp ring (clock tracker only). Useful for
    // spotting misconfigured ring depth or modem-side anomalies.
    virtual uint64_t fallback_anchor_count() const { return 0; }

    // EWMA-smoothed estimate of the modem's actual sample rate (clock
    // tracker only). Surfaced for operator visibility into rate-tracker
    // convergence; PID adapter returns 0.
    virtual double estimated_modem_rate() const { return 0.0; }
};

} // namespace openCREST
