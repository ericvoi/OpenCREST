#include "simulator/processing_time_stats.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

using openCREST::ProcessingTimeStats;

TEST(ProcessingTimeStats, AllZeroSnapshotReturnsSentinels) {
    ProcessingTimeStats stats;
    const auto snap = stats.snapshot();
    EXPECT_EQ(snap.count, 0u);
    EXPECT_EQ(snap.mean_us, 0.0);
    EXPECT_EQ(snap.p50_us, 0u);
    EXPECT_EQ(snap.p95_us, 0u);
    EXPECT_EQ(snap.p99_us, 0u);
    EXPECT_EQ(snap.max_us, 0u);
    EXPECT_EQ(snap.underrun_count, 0u);
}

TEST(ProcessingTimeStats, BucketsAreMonotonic) {
    // Sanity check on the log-spaced lookup: the bucket index must be
    // non-decreasing in the input duration.
    size_t last = 0;
    for (uint64_t us = 1; us <= 200'000; us *= 2) {
        const size_t b = ProcessingTimeStats::bucket_for(us);
        EXPECT_GE(b, last);
        last = b;
    }
}

TEST(ProcessingTimeStats, MeanMatchesUniformDistribution) {
    ProcessingTimeStats stats;
    std::mt19937 rng(42);
    std::uniform_int_distribution<uint64_t> dist(100, 200);
    uint64_t true_sum = 0;
    constexpr uint64_t N = 1000;
    for (uint64_t i = 0; i < N; ++i) {
        const uint64_t v = dist(rng);
        true_sum += v;
        stats.record(v);
    }
    const auto snap = stats.snapshot();
    EXPECT_EQ(snap.count, N);
    const double expected_mean = static_cast<double>(true_sum) /
                                 static_cast<double>(N);
    EXPECT_NEAR(snap.mean_us, expected_mean, 0.5);
}

TEST(ProcessingTimeStats, PercentilesRoughlyMatchUniformDistribution) {
    // Span wide enough to traverse many log buckets so percentile
    // resolution is meaningful. Asserts within ±15 % to avoid flakiness
    // (log-bucket resolution near 1 ms is ~5 %).
    ProcessingTimeStats stats;
    std::vector<uint64_t> samples;
    samples.reserve(10'000);
    std::mt19937 rng(7);
    std::uniform_int_distribution<uint64_t> dist(50, 5'000);
    for (size_t i = 0; i < 10'000; ++i) {
        const uint64_t v = dist(rng);
        samples.push_back(v);
        stats.record(v);
    }
    std::sort(samples.begin(), samples.end());
    const auto expect_pct = [&](double pct) {
        const size_t idx = static_cast<size_t>(samples.size() * pct);
        return samples[idx];
    };
    const auto snap = stats.snapshot();
    EXPECT_NEAR(snap.p50_us, expect_pct(0.50), expect_pct(0.50) * 0.15);
    EXPECT_NEAR(snap.p95_us, expect_pct(0.95), expect_pct(0.95) * 0.15);
    EXPECT_NEAR(snap.p99_us, expect_pct(0.99), expect_pct(0.99) * 0.15);
    EXPECT_EQ(snap.max_us, samples.back());
}

TEST(ProcessingTimeStats, UnderrunCountIncrementsAboveDeadline) {
    ProcessingTimeStats stats;
    const uint64_t deadline = 512;
    // 5 under deadline, 3 over.
    for (uint64_t v : {100u, 200u, 300u, 400u, 500u})
        stats.record(v, deadline);
    for (uint64_t v : {600u, 1000u, 5000u})
        stats.record(v, deadline);

    const auto snap = stats.snapshot();
    EXPECT_EQ(snap.count, 8u);
    EXPECT_EQ(snap.underrun_count, 3u);
}

TEST(ProcessingTimeStats, DeadlineZeroDisablesUnderrunPath) {
    ProcessingTimeStats stats;
    // deadline=0 must never increment underrun_count, regardless of v.
    for (uint64_t v : {1u, 10'000u, 99'999u})
        stats.record(v, /*deadline_us=*/0);
    EXPECT_EQ(stats.snapshot().underrun_count, 0u);
}
