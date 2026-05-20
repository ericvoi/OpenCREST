#include "core/tx_start_estimator.hpp"

namespace openCREST {

TxStartEstimator::TxStartEstimator(uint32_t modem_sample_rate)
    : modem_sample_rate_(modem_sample_rate)
{}

void TxStartEstimator::on_tx_entry() {
    cumulative_samples_ = 0;
    tightest_.reset();
}

void TxStartEstimator::on_tx_packet(uint16_t samples, time_point arrival_now) {
    cumulative_samples_ += samples;

    // T_lower_bound = arrival_now - cumulative_samples / Fs
    // Build the duration via microseconds so the arithmetic stays in
    // integer space and the rounding is consistent with the test
    // tolerance (±1 µs).
    const int64_t elapsed_us =
        static_cast<int64_t>((cumulative_samples_ * 1'000'000ull) /
                              modem_sample_rate_);
    const time_point t_lower = arrival_now - std::chrono::microseconds(elapsed_us);

    if (!tightest_ || t_lower > *tightest_) {
        tightest_ = t_lower;
    }
}

} // namespace openCREST
