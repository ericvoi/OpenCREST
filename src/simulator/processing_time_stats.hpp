#pragma once
#include <array>
#include <atomic>
#include <cstdint>

namespace openCREST {

// Lock-free per-batch processing-time histogram, fed from
// SourceWorker::process_available() with one record() per source-batch.
//
// Buckets are log-spaced from 1 µs to 100 ms across 256 slots; record()
// is allocation-free and ~10 ns. Snapshots are computed off the hot path
// after the engine has been joined, so the snapshot method itself is not
// thread-safe with concurrent record() — callers must order shutdown
// before snapshot().
class ProcessingTimeStats {
public:
    // Bucket count is fixed at compile time so storage is a flat array
    // of atomics — no allocation, no resize, cache-friendly.
    static constexpr size_t kBucketCount = 256;

    // Lower bound (µs) of the log-spaced range. 1 µs catches the
    // fastest plausible per-batch tick on bare metal.
    static constexpr double kMinUs = 1.0;
    // Upper bound (µs) of the log-spaced range. 100 ms safely contains
    // any plausible underrun before the modem itself would have starved.
    static constexpr double kMaxUs = 100'000.0;

    struct Snapshot {
        uint64_t count;
        double   mean_us;
        uint64_t p50_us;
        uint64_t p95_us;
        uint64_t p99_us;
        uint64_t max_us;
        uint64_t underrun_count;
    };

    ProcessingTimeStats() = default;
    ProcessingTimeStats(const ProcessingTimeStats&)            = delete;
    ProcessingTimeStats& operator=(const ProcessingTimeStats&) = delete;

    // Record one observed duration. Increments an underrun counter when
    // duration_us > deadline_us; pass deadline_us = 0 to disable that
    // path entirely.
    void record(uint64_t duration_us, uint64_t deadline_us = 0);

    Snapshot snapshot() const;

    // Map a duration to a histogram bucket index (clamped to range).
    // Exposed for tests so they can verify bucketing without inspecting
    // private state.
    static size_t bucket_for(uint64_t duration_us);

    // Inverse of bucket_for — the representative µs value at bucket
    // boundary i. Exposed for tests.
    static uint64_t bucket_lower_bound(size_t bucket_idx);

private:
    std::array<std::atomic<uint64_t>, kBucketCount> buckets_{};
    std::atomic<uint64_t> count_{0};
    std::atomic<uint64_t> sum_us_{0};
    std::atomic<uint64_t> max_us_{0};
    std::atomic<uint64_t> underrun_count_{0};
};

} // namespace openCREST
