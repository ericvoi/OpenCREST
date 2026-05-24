#pragma once
#include <chrono>
#include <cstdint>
#include <optional>

namespace openCREST {

// Lower-bound estimator for when the modem actually began its current
// TX burst, refined with each TX-stream packet received.
//
// At host-time T with N cumulative samples received, the modem must have
// started producing audio no later than T - N/Fs (USB transport delay
// can only add to T, so this is a true lower bound). The estimator
// keeps the maximum of this lower bound across observations — the
// tightest (latest) bound, closest to the true TX start from below.
//
// has_observation() is false until the first packet arrives; callers
// must seed cold start themselves (SourceWorker uses
// steady_clock::now()).
//
// Thread-safety: owned by a single ModemIO; all methods on that thread.
class TxStartEstimator {
public:
    using clock      = std::chrono::steady_clock;
    using time_point = clock::time_point;

    // `modem_sample_rate` is the modem's DAC sample rate (e.g. 500000),
    // used to convert sample counts to wall-clock durations.
    explicit TxStartEstimator(uint32_t modem_sample_rate);

    // Called on every *→TX state transition. Clears all observations.
    void on_tx_entry();

    // Called for every TX data packet accepted from the modem.
    // `samples` is the audio sample count in that packet (typically 255);
    // `arrival_now` is the host time at which it was recv'd.
    void on_tx_packet(uint16_t samples, time_point arrival_now);

    bool has_observation() const { return tightest_.has_value(); }

    std::optional<time_point> earliest_start_time() const { return tightest_; }

private:
    uint32_t                   modem_sample_rate_;
    uint64_t                   cumulative_samples_ = 0;
    std::optional<time_point>  tightest_;
};

} // namespace openCREST
