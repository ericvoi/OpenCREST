#pragma once
#include <chrono>
#include <cstdint>
#include "protocol/packets.hpp"

namespace openCREST {

// Abstract host-side estimator for a modem's RX audio-buffer fill level.
//
// Hosts the pacing decision (when to send the next host→modem RX data
// packet) and a query for the current extrapolated fill level (used by
// the channel pipeline to align message arrival times).
//
// Two implementations live behind this interface:
//
//   - PidFillTracker      (default; wraps the legacy BufferPacer)
//   - ClockFillTracker    (compile-time alternative; anchors fill to the
//                          host-side send timestamp of the firmware-named
//                          fill_reference_id and extrapolates from there)
//
// Selected at compile time via OPENCRIEST_USE_CLOCK_FILL_TRACKER. Both
// implementations always build — only the factory in ModemIO differs.
//
// Thread-safety: instances are owned by a single ModemIO; all methods are
// called from that one thread. No internal locking required.
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
    // `packet_id` is the wire-level packet_id (mirrors firmware
    // rx_expected_id counter); `samples` is the audio sample count in that
    // packet (255 in steady state).
    virtual void on_packet_sent(uint16_t packet_id,
                                uint16_t samples,
                                time_point now) = 0;

    // True if a packet should be sent at `now`. Internally advances the
    // schedule by one period on return-true (callers must respect that).
    virtual bool should_send(time_point now) = 0;

    // Absolute time of the next scheduled send. Callers may sleep until
    // this point to avoid busy-waiting between sends.
    virtual time_point next_send_time() const = 0;

    // Extrapolated current fill level in samples.
    virtual uint32_t estimated_fill(time_point now) const = 0;

    // Clear all state. Called on every modem state transition (mirrors the
    // firmware's resetHil() which zeros next_packet_index on each entry).
    virtual void reset() = 0;

    // Diagnostics (default zero so PID adapter doesn't need to override).
    // Clock tracker returns the cumulative count of status packets whose
    // fill_reference_id was not in the send-timestamp ring — useful for
    // operators to spot misconfigured ring depth or modem-side anomalies.
    virtual uint64_t fallback_anchor_count() const { return 0; }

    // Clock tracker returns its EWMA-smoothed estimate of the modem's
    // actual sample rate. PID adapter has no equivalent state and returns
    // 0; surfaced for operator visibility into rate-tracker convergence.
    virtual double estimated_modem_rate() const { return 0.0; }
};

} // namespace openCREST
