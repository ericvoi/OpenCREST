#pragma once
#include <chrono>
#include <cstdint>
#include <optional>

namespace openCREST {

// Lower-bound estimator for "when did the modem actually start its
// current TX burst", refined per TX-stream packet received from that
// modem.
//
// Mechanism. The modem produces audio at its DAC sample rate Fs. If by
// host-time T we have received N cumulative samples in this TX session,
// then the modem must have begun producing audio at host-time no later
// than T - N/Fs (USB transport delay only adds to T, never subtracts —
// so this is a true lower bound). The estimator stores the maximum of
// this lower bound across all observations; the max is the *tightest*
// (latest) bound, closest to the true TX start from below.
//
// First-packet seed: with no observations, has_observation() returns
// false and earliest_start_time() is empty. Callers must decide what to
// do at cold start (SourceWorker uses steady_clock::now() per
// /home/ern/.claude/plans/i-am-going-through-functional-comet.md).
//
// Thread-safety: instances are owned by a single ModemIO; all methods
// are called from that one thread.
class TxStartEstimator {
public:
    using clock      = std::chrono::steady_clock;
    using time_point = clock::time_point;

    // `modem_sample_rate` is the modem's DAC sample rate from
    // CalibrationData (e.g. 500000). Used to convert sample counts to
    // wall-clock durations.
    explicit TxStartEstimator(uint32_t modem_sample_rate);

    // Called on every RX→TX (or *→TX) state transition. Clears all
    // observations so the next message starts with a fresh count.
    void on_tx_entry();

    // Called from ModemIO::process_tx_packet for every TX data packet
    // accepted from the modem. `samples` is the audio sample count in
    // that packet (typically 255); `arrival_now` is the host time at
    // which the packet was successfully recv'd.
    void on_tx_packet(uint16_t samples, time_point arrival_now);

    bool has_observation() const { return tightest_.has_value(); }

    std::optional<time_point> earliest_start_time() const { return tightest_; }

private:
    uint32_t                   modem_sample_rate_;
    uint64_t                   cumulative_samples_ = 0;
    std::optional<time_point>  tightest_;
};

} // namespace openCREST
