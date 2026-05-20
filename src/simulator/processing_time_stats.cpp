#include "simulator/processing_time_stats.hpp"

#include <algorithm>
#include <cmath>

namespace openCREST {

namespace {

// Pre-computed log scaling: bucket_idx = log(us) * kScale + kBias,
// solved so bucket 0 covers [0, kMinUs) and bucket kBucketCount-1
// holds the open-ended >= kMaxUs tail.
constexpr double log_min() { return 0.0; } // log(kMinUs=1) = 0
const     double kLogMax = std::log(ProcessingTimeStats::kMaxUs);
const     double kScale  =
    static_cast<double>(ProcessingTimeStats::kBucketCount - 1) /
    (kLogMax - log_min());

} // namespace

size_t ProcessingTimeStats::bucket_for(uint64_t duration_us) {
    if (duration_us == 0) return 0;
    const double lg = std::log(static_cast<double>(duration_us));
    // Linear log→bucket mapping, clamped at both ends.
    const double idx_f = lg * kScale;
    if (idx_f <= 0.0)                      return 0;
    if (idx_f >= static_cast<double>(kBucketCount - 1))
        return kBucketCount - 1;
    return static_cast<size_t>(idx_f);
}

uint64_t ProcessingTimeStats::bucket_lower_bound(size_t bucket_idx) {
    if (bucket_idx == 0) return 0;
    const double lg = static_cast<double>(bucket_idx) / kScale;
    const double us = std::exp(lg);
    return static_cast<uint64_t>(us);
}

void ProcessingTimeStats::record(uint64_t duration_us, uint64_t deadline_us) {
    const size_t b = bucket_for(duration_us);
    buckets_[b].fetch_add(1, std::memory_order_relaxed);
    count_.fetch_add(1,         std::memory_order_relaxed);
    sum_us_.fetch_add(duration_us, std::memory_order_relaxed);

    // Atomic max via CAS loop. Bounded by the number of records that
    // happen to be in flight simultaneously; with one source thread
    // calling this serially the loop almost always completes on the
    // first try.
    uint64_t prev = max_us_.load(std::memory_order_relaxed);
    while (duration_us > prev &&
           !max_us_.compare_exchange_weak(prev, duration_us,
                                          std::memory_order_relaxed)) {
        // prev refreshed by CAS; retry.
    }

    if (deadline_us > 0 && duration_us > deadline_us) {
        underrun_count_.fetch_add(1, std::memory_order_relaxed);
    }
}

ProcessingTimeStats::Snapshot ProcessingTimeStats::snapshot() const {
    Snapshot s{};
    s.count          = count_.load(std::memory_order_relaxed);
    s.max_us         = max_us_.load(std::memory_order_relaxed);
    s.underrun_count = underrun_count_.load(std::memory_order_relaxed);

    if (s.count == 0) {
        s.mean_us = 0.0;
        s.p50_us = s.p95_us = s.p99_us = 0;
        return s;
    }

    const uint64_t sum = sum_us_.load(std::memory_order_relaxed);
    s.mean_us = static_cast<double>(sum) / static_cast<double>(s.count);

    // Percentile via cumulative bucket scan. The bucket lower-bound is
    // the reported percentile value — one bucket of granularity, which
    // the plan permits ("match within 1 bucket").
    const uint64_t target_p50 = (s.count *  50 + 99) / 100;
    const uint64_t target_p95 = (s.count *  95 + 99) / 100;
    const uint64_t target_p99 = (s.count *  99 + 99) / 100;

    uint64_t cumulative = 0;
    bool got_p50 = false, got_p95 = false, got_p99 = false;
    for (size_t i = 0; i < kBucketCount; ++i) {
        cumulative += buckets_[i].load(std::memory_order_relaxed);
        if (!got_p50 && cumulative >= target_p50) {
            s.p50_us = bucket_lower_bound(i);
            got_p50  = true;
        }
        if (!got_p95 && cumulative >= target_p95) {
            s.p95_us = bucket_lower_bound(i);
            got_p95  = true;
        }
        if (!got_p99 && cumulative >= target_p99) {
            s.p99_us = bucket_lower_bound(i);
            got_p99  = true;
            break;
        }
    }
    return s;
}

} // namespace openCREST
