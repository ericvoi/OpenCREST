// TxStartEstimator pins the math of "how early could the modem have
// started its current TX burst, given every TX-stream packet I've
// received so far". The estimator returns the *tightest* lower bound
// (latest such time) since each additional packet only ever proves the
// modem started no later than `arrival_now - cumulative_samples / Fs`.
#include <gtest/gtest.h>
#include "core/tx_start_estimator.hpp"

using openCREST::TxStartEstimator;
using est_clock  = TxStartEstimator::clock;
using time_point = TxStartEstimator::time_point;

namespace {
constexpr uint32_t FS = 500'000;  // modem sample rate, samples/sec

// micros() helper — build a duration from microseconds
constexpr auto us(int64_t n) {
    return std::chrono::microseconds(n);
}
} // namespace

// Test 4: After construction, no observation exists. The estimator must
// not return a phantom bound that could mis-align the first message.
TEST(TxStartEstimator, InitialStateNoObservation) {
    TxStartEstimator est(FS);
    EXPECT_FALSE(est.has_observation());
    EXPECT_FALSE(est.earliest_start_time().has_value());
}

// Test 5: One packet of N samples arriving at time T proves the modem
// began transmitting no later than T - N/Fs.
TEST(TxStartEstimator, FirstPacketSeedsEstimate) {
    TxStartEstimator est(FS);
    const time_point T = est_clock::now();
    const uint16_t samples = 255;

    est.on_tx_packet(samples, T);

    ASSERT_TRUE(est.has_observation());
    const time_point expected = T - us((static_cast<int64_t>(samples) * 1'000'000) / FS);
    const auto bound = *est.earliest_start_time();
    // Allow ±1 µs tolerance for integer rounding in samples→duration.
    EXPECT_LE(std::chrono::abs(bound - expected), us(1));
}

// Test 6: Subsequent packets at steady cadence MUST NOT loosen the
// bound. They can only refine it later (tighter lower bound = larger
// timestamp). Even if the math sometimes regresses due to USB jitter,
// the stored bound is the maximum across observations.
TEST(TxStartEstimator, MonotonicTightening) {
    TxStartEstimator est(FS);
    const time_point T0 = est_clock::now();
    const uint16_t samples = 255;
    const auto pkt_period = us((static_cast<int64_t>(samples) * 1'000'000) / FS);

    est.on_tx_packet(samples, T0);
    const auto first_bound = *est.earliest_start_time();

    // Perfect cadence: packet N arrives at T0 + N·pkt_period with N+1
    // cumulative packets seen → bound shifts to T0 - pkt_period each time
    // and stays exactly equal to first_bound. We assert non-regression.
    for (int n = 1; n < 10; ++n) {
        est.on_tx_packet(samples, T0 + n * pkt_period);
        const auto bound = *est.earliest_start_time();
        EXPECT_GE(bound, first_bound);
    }
}

// Test 7: An out-of-order or jittered packet whose computed lower bound
// is *earlier* than a prior observation MUST NOT replace the tighter
// bound. We keep the max across observations.
TEST(TxStartEstimator, MaxOverObservationsIsTightest) {
    TxStartEstimator est(FS);
    const time_point T0 = est_clock::now();

    // First packet: 255 samples at T0 → bound = T0 - 510 µs
    est.on_tx_packet(255, T0);
    const auto tight = *est.earliest_start_time();

    // Synthesize a "delayed" second packet whose computed lower bound
    // would be earlier than `tight` (e.g. samples advance faster than
    // wall clock, simulating USB stall). cumulative = 510 samples,
    // arrival = T0 + 100 µs (vs. expected 510 µs of wall clock).
    // T_lower = (T0 + 100us) - 1020us = T0 - 920us, which is *earlier*
    // than the first bound (T0 - 510us). We must NOT regress.
    est.on_tx_packet(255, T0 + us(100));
    const auto after = *est.earliest_start_time();
    EXPECT_EQ(after, tight);
}

// Test 8: on_tx_entry() resets all state. Stale data from a prior TX
// burst MUST NOT leak into the next message's alignment.
TEST(TxStartEstimator, ResetOnTxEntryClearsAll) {
    TxStartEstimator est(FS);
    const time_point T0 = est_clock::now();

    est.on_tx_packet(255, T0);
    est.on_tx_packet(255, T0 + us(510));
    ASSERT_TRUE(est.has_observation());

    est.on_tx_entry();
    EXPECT_FALSE(est.has_observation());
    EXPECT_FALSE(est.earliest_start_time().has_value());

    // After reset, the next observation seeds fresh — not aliased to
    // prior cumulative_samples.
    const time_point T_new = T0 + std::chrono::milliseconds(100);
    est.on_tx_packet(255, T_new);
    const auto bound = *est.earliest_start_time();
    const time_point expected = T_new - us(510);
    EXPECT_LE(std::chrono::abs(bound - expected), us(1));
}
