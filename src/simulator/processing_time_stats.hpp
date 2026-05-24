#pragma once
#include <array>
#include <atomic>
#include <cstdint>

namespace openCREST {

// Lock-free per-batch processing-time histogram, fed by
// SourceWorker::process_available() with one record() per source-batch.
//
// Buckets are log-spaced from 1 µs to 100 ms across 256 slots; record()
// is allocation-free and ~10 ns. snapshot() is not thread-safe against
// concurrent record() — callers must order shutdown first.
class ProcessingTimeStats {
public:
    // Fixed at compile time so storage is a flat atomic array.
    static constexpr size_t kBucketCount = 256;

    // Log-spaced range bounds: 1 µs catches the fastest plausible per-batch
    // tick; 100 ms covers any plausible underrun before the modem starves.
    static constexpr double kMinUs = 1.0;
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

    // Increments underrun_count when duration_us > deadline_us; pass
    // deadline_us = 0 to disable that check.
    void record(uint64_t duration_us, uint64_t deadline_us = 0);

    Snapshot snapshot() const;

    // Map a duration to a histogram bucket index (clamped to range).
    // Exposed for tests.
    static size_t bucket_for(uint64_t duration_us);

    // Representative µs value at bucket boundary i. Exposed for tests.
    static uint64_t bucket_lower_bound(size_t bucket_idx);

private:
    std::array<std::atomic<uint64_t>, kBucketCount> buckets_{};
    std::atomic<uint64_t> count_{0};
    std::atomic<uint64_t> sum_us_{0};
    std::atomic<uint64_t> max_us_{0};
    std::atomic<uint64_t> underrun_count_{0};
};

} // namespace openCREST
