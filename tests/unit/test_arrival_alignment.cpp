// Tests for SourceWorker::compute_arrival_aligned_gap.
//
// Math under test:
//   T_play(prior_wm) ≈ now + (prior_wm − read_pos + rx_ring_depth + F_B) / Fs_B
//   Want T_play(write_origin) == T_tx_start + propagation_delay_s
//   write_origin = prior_wm + gap_samples
//   ⇒  gap_samples = (T_tx_start + prop − now) × Fs_B
//                  − (prior_wm − read_pos)
//                  − rx_ring_depth
//                  − F_B
#include <gtest/gtest.h>
#include "channel/source_worker.hpp"

using openCREST::SourceWorker;
using sw_clock   = std::chrono::steady_clock;
using time_point = sw_clock::time_point;

namespace {
constexpr uint32_t FS_B = 500'000;
constexpr auto us(int64_t n) { return std::chrono::microseconds(n); }
constexpr auto ms(int64_t n) { return std::chrono::milliseconds(n); }
} // namespace

// Empty pipeline and zero propagation: gap is zero when T_tx_start == now.
TEST(ArrivalAlignment, GapZeroAtTxStartEqualsNowAndEmptyPipeline) {
    const time_point now = sw_clock::now();
    const size_t gap = SourceWorker::compute_arrival_aligned_gap(
        /*T_tx_start*/ now,
        /*prop_s*/     0.0,
        now,
        FS_B,
        /*F_B*/        0,
        /*prior_wm*/   0,
        /*read_pos*/   0,
        /*rx_ring*/    0);
    EXPECT_EQ(gap, 0u);
}

// With realistic pipeline depths and 333 ms propagation, the chosen gap
// places the first sample at T_tx_start + propagation within ±1 packet.
TEST(ArrivalAlignment, GapPlacesFirstSampleAtTxStartPlusPropagation) {
    const time_point now = sw_clock::now();
    const time_point T_tx_start = now - ms(10);  // modem already running 10 ms
    const double prop_s = 0.333;                  // 500 m / 1500 m/s
    const uint32_t F_B  = 8000;                   // receiver buffer ~50% full
    const size_t prior_wm = 100'000;
    const size_t read_pos = 50'000;
    const size_t rx_ring  = 255;

    const size_t gap = SourceWorker::compute_arrival_aligned_gap(
        T_tx_start, prop_s, now, FS_B, F_B,
        prior_wm, read_pos, rx_ring);

    // Reconstruct expected write_origin: prior_wm + gap.
    // Expected modem-play-time of write_origin:
    //   now + (prior_wm + gap − read_pos + rx_ring + F_B) / Fs_B
    // Should equal T_tx_start + prop_s within ±1 packet (510 µs).
    const double play_seconds_from_now =
        static_cast<double>(prior_wm + gap - read_pos + rx_ring + F_B) /
        static_cast<double>(FS_B);
    const auto play_time = now +
        std::chrono::duration_cast<sw_clock::duration>(
            std::chrono::duration<double>(play_seconds_from_now));
    const auto target = T_tx_start +
        std::chrono::duration_cast<sw_clock::duration>(
            std::chrono::duration<double>(prop_s));

    const auto err_us = std::chrono::duration_cast<std::chrono::microseconds>(
        play_time - target);
    EXPECT_LE(std::abs(err_us.count()), 510);  // 1 packet at 500 kSPS
}

// If the ideal injection point is already in the past, the gap must
// clamp to 0 and the "late" out-parameter must fire.
TEST(ArrivalAlignment, LateMessageClampsGapToZero) {
    const time_point now = sw_clock::now();
    // Modem started 100 ms ago, propagation is 0 → message should
    // already be playing 100 ms ago. We can't go back in time.
    const time_point T_tx_start = now - ms(100);
    const double prop_s = 0.0;

    bool late = false;
    const size_t gap = SourceWorker::compute_arrival_aligned_gap(
        T_tx_start, prop_s, now, FS_B,
        /*F_B*/ 8000, /*prior_wm*/ 100, /*read_pos*/ 0,
        /*rx_ring*/ 0, &late);

    EXPECT_EQ(gap, 0u);
    EXPECT_TRUE(late);
}

// Cold-start: when T_tx_start == now the math stays stable (no NaN,
// gap stays bounded) regardless of pipeline depth or propagation.
TEST(ArrivalAlignment, EstimatorColdStartProducesBoundedGap) {
    const time_point now = sw_clock::now();
    const double prop_s = 0.0;
    const size_t prior_wm = 8192;
    const size_t read_pos = 0;
    const uint32_t F_B = 8192;
    const size_t rx_ring = 255;

    // With T_tx_start = now and a non-trivial pipeline depth, the
    // target play time is "now"; the actual earliest playable time is
    // "now + pipeline_depth/Fs". So the message is necessarily late,
    // gap clamped to 0.
    bool late = false;
    const size_t gap = SourceWorker::compute_arrival_aligned_gap(
        now, prop_s, now, FS_B, F_B,
        prior_wm, read_pos, rx_ring, &late);
    EXPECT_EQ(gap, 0u);
    EXPECT_TRUE(late);

    // With prop_s large enough to absorb the pipeline depth, the gap
    // becomes positive but bounded:
    //   gap ≈ prop_s × Fs − pipeline_depth
    const double large_prop = 0.5;  // 500 ms
    const size_t gap2 = SourceWorker::compute_arrival_aligned_gap(
        now, large_prop, now, FS_B, F_B,
        prior_wm, read_pos, rx_ring);
    const int64_t expected = static_cast<int64_t>(large_prop * FS_B)
                            - static_cast<int64_t>(prior_wm - read_pos)
                            - static_cast<int64_t>(rx_ring)
                            - static_cast<int64_t>(F_B);
    EXPECT_EQ(static_cast<int64_t>(gap2), expected);
}
