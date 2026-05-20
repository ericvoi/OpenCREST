#include <gtest/gtest.h>
#include "channel/pair_buffer.hpp"

#include <atomic>
#include <chrono>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

using openCREST::PairBuffer;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

// Drain `count` samples into `out`, looping read_advance until it returns 0
// or the count is satisfied. Used to flush the buffer in steady-state tests.
size_t drain_exact(PairBuffer& pb, std::vector<float>& out, size_t count) {
    out.assign(count, std::numeric_limits<float>::quiet_NaN());
    size_t got = 0;
    while (got < count) {
        const size_t n = pb.read_advance(out.data() + got, count - got);
        if (n == 0) break;
        got += n;
    }
    out.resize(got);
    return got;
}

} // namespace

// ===========================================================================
// Construction / capacity / initial state
// ===========================================================================

TEST(PairBuffer, CapacityRoundsUpToPowerOfTwo) {
    PairBuffer pb1(1000, 100);
    EXPECT_EQ(pb1.capacity(), 1024u);

    PairBuffer pb2(1024, 100);
    EXPECT_EQ(pb2.capacity(), 1024u);

    PairBuffer pb3(1, 0);
    EXPECT_EQ(pb3.capacity(), 1u);
}

TEST(PairBuffer, BaseDelayAccessible) {
    PairBuffer pb(4096, 333);
    EXPECT_EQ(pb.base_delay_samples(), 333u);
}

TEST(PairBuffer, InitialAvailableReadIsZero) {
    PairBuffer pb(4096, 100);
    EXPECT_EQ(pb.available_read(), 0u);
    EXPECT_EQ(pb.overflow_drops(), 0u);
}

TEST(PairBuffer, ReadOnEmptyBufferReturnsZero) {
    PairBuffer pb(4096, 100);
    std::vector<float> out(8, 1.0f);
    EXPECT_EQ(pb.read_advance(out.data(), 8), 0u);
}

// ===========================================================================
// First-message lifecycle: begin → scatter → commit → read
// ===========================================================================

TEST(PairBuffer, FirstMessageWriteOriginIsBaseDelay) {
    // Legacy / PID-mode policy (absolute_first_origin = false, the default):
    // after begin_message(0) on a fresh buffer, source sample 0 (with tap
    // delta 0) lands at receiver position base_delay. Reader sees base_delay
    // zeros first, then the scattered data.
    constexpr size_t base_delay = 16;
    PairBuffer pb(4096, base_delay);

    pb.begin_message(0);

    const float data[] = {1.0f, 2.0f, 3.0f, 4.0f};
    pb.scatter_add(0, data, 4);

    // No commit yet → reader sees nothing.
    EXPECT_EQ(pb.available_read(), 0u);

    pb.commit_source_progress(4);

    // Watermark = write_origin + 4 = base_delay + 4. read_pos = 0.
    EXPECT_EQ(pb.available_read(), base_delay + 4);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, base_delay + 4), base_delay + 4);

    // First base_delay samples are the propagation-delay silence.
    for (size_t i = 0; i < base_delay; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0f) << "i=" << i;
    }
    // Then the scattered data.
    EXPECT_FLOAT_EQ(out[base_delay + 0], 1.0f);
    EXPECT_FLOAT_EQ(out[base_delay + 1], 2.0f);
    EXPECT_FLOAT_EQ(out[base_delay + 2], 3.0f);
    EXPECT_FLOAT_EQ(out[base_delay + 3], 4.0f);
}

TEST(PairBuffer, FirstMessageAbsoluteFirstOriginIgnoresBaseDelay) {
    // Clock-tracker arrival-alignment policy: with absolute_first_origin =
    // true, the buffer does NOT auto-apply base_delay on the first message.
    // write_origin = caller-supplied gap, and the gap is expected to have
    // already incorporated the propagation delay (via the arrival-alignment
    // formula upstream in SourceWorker).
    //
    // Regression guard for the 25 m one-shot ranging bias: pre-fix,
    // PairBuffer auto-applied base_delay regardless of the caller's intent,
    // bypassing the arrival-alignment math and surfacing the receiver
    // modem's pre-fill backlog as a one-shot per-direction bias on each
    // PairBuffer's first message after simulator restart.
    constexpr size_t gap = 7;
    PairBuffer pb(4096, /*base_delay_samples=*/256);

    pb.begin_message(gap, /*absolute_first_origin=*/true);

    const float data[] = {1.0f, 2.0f, 3.0f, 4.0f};
    pb.scatter_add(0, data, 4);
    pb.commit_source_progress(4);

    // write_origin = gap (NOT base_delay_samples_ = 256).
    // Watermark = gap + 4 = 11. read_pos = 0.
    EXPECT_EQ(pb.available_read(), gap + 4);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, gap + 4), gap + 4);
    for (size_t i = 0; i < gap; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0f) << "i=" << i;
    }
    EXPECT_FLOAT_EQ(out[gap + 0], 1.0f);
    EXPECT_FLOAT_EQ(out[gap + 1], 2.0f);
    EXPECT_FLOAT_EQ(out[gap + 2], 3.0f);
    EXPECT_FLOAT_EQ(out[gap + 3], 4.0f);
}

TEST(PairBuffer, SubsequentMessageUnaffectedByAbsoluteFirstOriginFlag) {
    // The absolute_first_origin flag only controls the FIRST message's
    // policy. Once first_message_ has been consumed, subsequent messages
    // unconditionally use prior_watermark + gap regardless of the flag.
    PairBuffer pb(4096, /*base_delay_samples=*/100);

    // First message via legacy default → write_origin = 100.
    pb.begin_message(0);
    const float m1[] = {1.0f};
    pb.scatter_add(0, m1, 1);
    pb.commit_source_progress(1);
    EXPECT_EQ(pb.available_read(), 101u);  // 100 zeros + 1 sample

    // Second message with absolute_first_origin=true (a no-op once
    // first_message_ has been consumed). write_origin = prior_wm (101) + 5.
    pb.begin_message(5, /*absolute_first_origin=*/true);
    const float m2[] = {2.0f};
    pb.scatter_add(0, m2, 1);
    pb.commit_source_progress(1);
    EXPECT_EQ(pb.available_read(), 107u);  // 101 prior + 5 gap + 1 new sample
}

TEST(PairBuffer, ReadAdvanceClampsToAvailable) {
    PairBuffer pb(4096, 4);
    pb.begin_message(0);
    const float data[] = {7.0f, 8.0f, 9.0f};
    pb.scatter_add(0, data, 3);
    pb.commit_source_progress(3);

    // Available = 4 (delay) + 3 (data) = 7
    std::vector<float> out(100, -1.0f);
    EXPECT_EQ(pb.read_advance(out.data(), 100), 7u);
    EXPECT_EQ(pb.available_read(), 0u);
}

TEST(PairBuffer, PartialReadAdvancesPosition) {
    PairBuffer pb(4096, 0);
    pb.begin_message(0);
    const float data[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    pb.scatter_add(0, data, 5);
    pb.commit_source_progress(5);

    std::vector<float> out(2);
    EXPECT_EQ(pb.read_advance(out.data(), 2), 2u);
    EXPECT_FLOAT_EQ(out[0], 1.0f);
    EXPECT_FLOAT_EQ(out[1], 2.0f);
    EXPECT_EQ(pb.available_read(), 3u);

    EXPECT_EQ(pb.read_advance(out.data(), 2), 2u);
    EXPECT_FLOAT_EQ(out[0], 3.0f);
    EXPECT_FLOAT_EQ(out[1], 4.0f);

    EXPECT_EQ(pb.read_advance(out.data(), 2), 1u);
    EXPECT_FLOAT_EQ(out[0], 5.0f);
}

// ===========================================================================
// Multipath: scatter at non-zero offsets, accumulation, commit_extra
// ===========================================================================

TEST(PairBuffer, ScatterAtOffsetIsInvisibleUntilCommitExtra) {
    constexpr size_t base = 0;
    PairBuffer pb(4096, base);
    pb.begin_message(0);

    // Scatter only at offset 5 (a multipath tap with delta=5).
    const float data[] = {1.0f, 1.0f, 1.0f};
    pb.scatter_add(5, data, 3);

    // No commit_source_progress yet.
    EXPECT_EQ(pb.available_read(), 0u);

    // commit_source_progress(0): nothing source-side.
    pb.commit_source_progress(0);
    EXPECT_EQ(pb.available_read(), 0u);

    // commit_extra(8) exposes positions [0, 8). Tap data at [5,8).
    pb.commit_extra(8);
    EXPECT_EQ(pb.available_read(), 8u);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, 8), 8u);
    const float expected[] = {0,0,0,0,0, 1,1,1};
    for (size_t i = 0; i < 8; ++i) {
        EXPECT_FLOAT_EQ(out[i], expected[i]) << "i=" << i;
    }
}

TEST(PairBuffer, MultipathTapsAccumulate) {
    // Two taps: direct (delta=0, gain=1) + reflection (delta=2, gain=1).
    // Source samples [1,1,1] processed.
    constexpr size_t base = 0;
    PairBuffer pb(4096, base);
    pb.begin_message(0);

    const float ones[] = {1.0f, 1.0f, 1.0f};
    pb.scatter_add(0, ones, 3);  // direct path
    pb.scatter_add(2, ones, 3);  // delta=2 tap

    pb.commit_source_progress(3);
    pb.commit_extra(2);  // expose multipath tail (max_tap_delta = 2)

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, 5), 5u);

    // Source samples s0,s1,s2 = 1,1,1.
    // Tap0 (delta=0): writes positions [0,1,2] with 1 each.
    // Tap1 (delta=2): writes positions [2,3,4] with 1 each.
    // Position 0: tap0/s0          → 1
    // Position 1: tap0/s1          → 1
    // Position 2: tap0/s2 + tap1/s0 → 2
    // Position 3: tap1/s1          → 1
    // Position 4: tap1/s2          → 1
    EXPECT_FLOAT_EQ(out[0], 1.0f);
    EXPECT_FLOAT_EQ(out[1], 1.0f);
    EXPECT_FLOAT_EQ(out[2], 2.0f);
    EXPECT_FLOAT_EQ(out[3], 1.0f);
    EXPECT_FLOAT_EQ(out[4], 1.0f);
}

TEST(PairBuffer, ScatterAddAccumulatesNonOverwriting) {
    // Calling scatter_add twice at the same offset adds (does not overwrite).
    PairBuffer pb(4096, 0);
    pb.begin_message(0);

    const float a[] = {1.0f, 2.0f, 3.0f};
    const float b[] = {10.0f, 20.0f, 30.0f};
    pb.scatter_add(0, a, 3);
    pb.scatter_add(0, b, 3);
    pb.commit_source_progress(3);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, 3), 3u);
    EXPECT_FLOAT_EQ(out[0], 11.0f);
    EXPECT_FLOAT_EQ(out[1], 22.0f);
    EXPECT_FLOAT_EQ(out[2], 33.0f);
}

// ===========================================================================
// commit_source_progress: monotonic, idempotent, only-grows
// ===========================================================================

TEST(PairBuffer, CommitSourceProgressIsMonotonic) {
    PairBuffer pb(4096, 0);
    pb.begin_message(0);

    const float data[] = {1, 2, 3, 4, 5, 6, 7, 8};
    pb.scatter_add(0, data, 8);

    pb.commit_source_progress(5);
    EXPECT_EQ(pb.available_read(), 5u);

    // Smaller value: no-op.
    pb.commit_source_progress(3);
    EXPECT_EQ(pb.available_read(), 5u);

    // Same value: no-op.
    pb.commit_source_progress(5);
    EXPECT_EQ(pb.available_read(), 5u);

    // Larger value: advances.
    pb.commit_source_progress(8);
    EXPECT_EQ(pb.available_read(), 8u);
}

TEST(PairBuffer, CommitExtraAdvancesPastSourceCommit) {
    PairBuffer pb(4096, 0);
    pb.begin_message(0);
    const float data[] = {1, 2, 3};
    pb.scatter_add(0, data, 3);

    pb.commit_source_progress(3);
    EXPECT_EQ(pb.available_read(), 3u);

    pb.commit_extra(4);  // simulate exposing 4-sample multipath tail
    EXPECT_EQ(pb.available_read(), 7u);

    pb.commit_extra(0);  // no-op
    EXPECT_EQ(pb.available_read(), 7u);

    pb.commit_extra(2);
    EXPECT_EQ(pb.available_read(), 9u);
}

// ===========================================================================
// read_advance zeroes consumed slots
// ===========================================================================

TEST(PairBuffer, ReadAdvanceZeroesConsumedSlotsForReuse) {
    // Scatter, read, scatter again at the same physical slot via wrap; the
    // second scatter must accumulate from 0, not from prior data.
    constexpr size_t cap = 16;
    PairBuffer pb(cap, 0);
    pb.begin_message(0);

    const float a[] = {1, 2, 3, 4, 5, 6, 7, 8};
    pb.scatter_add(0, a, 8);
    pb.commit_source_progress(8);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, 8), 8u);
    for (size_t i = 0; i < 8; ++i) EXPECT_FLOAT_EQ(out[i], static_cast<float>(i + 1));

    // Now write 8 more samples — these wrap to the same physical slots.
    // If the slots weren't zeroed, accumulation would corrupt the new data.
    const float b[] = {100, 200, 300, 400, 500, 600, 700, 800};
    pb.scatter_add(8, b, 8);
    pb.commit_source_progress(16);

    out.clear();
    ASSERT_EQ(drain_exact(pb, out, 8), 8u);
    for (size_t i = 0; i < 8; ++i)
        EXPECT_FLOAT_EQ(out[i], static_cast<float>((i + 1) * 100));
}

// ===========================================================================
// Wrap-around correctness across many cycles
// ===========================================================================

TEST(PairBuffer, ManyWrapCyclesPreserveStreamData) {
    constexpr size_t cap = 64;
    PairBuffer pb(cap, 0);
    pb.begin_message(0);

    constexpr size_t batch = 16;
    constexpr size_t cycles = 20;

    std::vector<float> tmp(batch);
    size_t source_pos = 0;

    for (size_t c = 0; c < cycles; ++c) {
        std::vector<float> in(batch);
        for (size_t i = 0; i < batch; ++i)
            in[i] = static_cast<float>(c * batch + i);

        pb.scatter_add(source_pos, in.data(), batch);
        source_pos += batch;
        pb.commit_source_progress(source_pos);

        ASSERT_EQ(drain_exact(pb, tmp, batch), batch);
        for (size_t i = 0; i < batch; ++i) {
            EXPECT_FLOAT_EQ(tmp[i], static_cast<float>(c * batch + i))
                << "cycle=" << c << " i=" << i;
        }
    }
}

// ===========================================================================
// Overflow protection: writes past read_pos + capacity drop
// ===========================================================================

TEST(PairBuffer, OverflowDropsExcessSamplesAndCounts) {
    constexpr size_t cap = 16;
    PairBuffer pb(cap, 0);
    pb.begin_message(0);

    // Scatter past capacity — must drop, not write into already-read region.
    std::vector<float> in(cap + 8, 1.0f);
    pb.scatter_add(0, in.data(), cap + 8);
    pb.commit_source_progress(cap + 8);

    EXPECT_GT(pb.overflow_drops(), 0u);

    // Reader can still read up to capacity worth of valid data.
    std::vector<float> out(cap);
    EXPECT_EQ(pb.read_advance(out.data(), cap), cap);
}

// ===========================================================================
// Multi-message back-to-back
// ===========================================================================

TEST(PairBuffer, BackToBackMessagesAppendInReceiverTime) {
    constexpr size_t base = 4;
    PairBuffer pb(4096, base);

    // Message 1
    pb.begin_message(0);
    const float m1[] = {1, 1, 1, 1};
    pb.scatter_add(0, m1, 4);
    pb.commit_source_progress(4);
    pb.commit_extra(0);  // no multipath tail

    // Watermark = base + 4
    EXPECT_EQ(pb.available_read(), base + 4);

    // Message 2 begins before reader has drained
    pb.begin_message(0);  // gap = 0, back-to-back

    const float m2[] = {2, 2, 2, 2};
    pb.scatter_add(0, m2, 4);
    pb.commit_source_progress(4);

    // Watermark advanced to (prior_watermark + 4) = base + 8
    EXPECT_EQ(pb.available_read(), base + 8);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, base + 8), base + 8);

    // base zeros (initial propagation delay), then msg1 (4×1), then msg2 (4×2)
    for (size_t i = 0; i < base; ++i)        EXPECT_FLOAT_EQ(out[i], 0.0f) << "i=" << i;
    for (size_t i = base; i < base + 4; ++i) EXPECT_FLOAT_EQ(out[i], 1.0f) << "i=" << i;
    for (size_t i = base + 4; i < base + 8; ++i) EXPECT_FLOAT_EQ(out[i], 2.0f) << "i=" << i;
}

TEST(PairBuffer, MessageWithGapInsertsSilenceBetweenMessages) {
    constexpr size_t base = 0;
    PairBuffer pb(4096, base);

    pb.begin_message(0);
    const float m1[] = {1, 1};
    pb.scatter_add(0, m1, 2);
    pb.commit_source_progress(2);

    // 5-sample inter-message gap
    pb.begin_message(5);

    const float m2[] = {2, 2};
    pb.scatter_add(0, m2, 2);
    pb.commit_source_progress(2);

    // Watermark = (m1_watermark = 2) + gap(5) + m2(2) = 9
    EXPECT_EQ(pb.available_read(), 9u);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, 9), 9u);
    const float expected[] = {1,1, 0,0,0,0,0, 2,2};
    for (size_t i = 0; i < 9; ++i) EXPECT_FLOAT_EQ(out[i], expected[i]) << "i=" << i;
}

TEST(PairBuffer, ConcurrentNewMessageBeginsWhileReaderDrains) {
    // Reader is mid-drain of message 1 when message 2 begins. Reader must
    // continue receiving msg1 tail then naturally see msg2.
    constexpr size_t base = 0;
    PairBuffer pb(4096, base);

    pb.begin_message(0);
    const float m1[] = {1, 2, 3, 4, 5, 6, 7, 8};
    pb.scatter_add(0, m1, 8);
    pb.commit_source_progress(8);

    std::vector<float> out(4);
    ASSERT_EQ(pb.read_advance(out.data(), 4), 4u);
    EXPECT_FLOAT_EQ(out[0], 1.0f);
    EXPECT_FLOAT_EQ(out[3], 4.0f);

    // Now message 2 begins (reader hasn't finished msg1).
    pb.begin_message(0);
    const float m2[] = {100, 200, 300};
    pb.scatter_add(0, m2, 3);
    pb.commit_source_progress(3);

    // Available should now = (msg1 remainder 4) + (msg2 = 3) = 7
    EXPECT_EQ(pb.available_read(), 7u);

    std::vector<float> rest;
    ASSERT_EQ(drain_exact(pb, rest, 7), 7u);
    EXPECT_FLOAT_EQ(rest[0], 5.0f);
    EXPECT_FLOAT_EQ(rest[1], 6.0f);
    EXPECT_FLOAT_EQ(rest[2], 7.0f);
    EXPECT_FLOAT_EQ(rest[3], 8.0f);
    EXPECT_FLOAT_EQ(rest[4], 100.0f);
    EXPECT_FLOAT_EQ(rest[5], 200.0f);
    EXPECT_FLOAT_EQ(rest[6], 300.0f);
}

TEST(PairBuffer, OverlapBetweenMessagesAccumulatesViaScatter) {
    // If msg1's tail (via commit_extra) has been published, msg2's
    // begin_message places the new origin past that tail — the regions do
    // not overlap. But within a single message, two scatter_adds at
    // overlapping receiver positions accumulate. (Already covered above —
    // this test doubles down on the inter-message non-overlap invariant.)
    PairBuffer pb(4096, 0);

    pb.begin_message(0);
    const float m1[] = {1, 1, 1};
    pb.scatter_add(0, m1, 3);     // msg1 commits source positions 0..3
    pb.commit_source_progress(3);
    pb.commit_extra(2);           // multipath tail of length 2: watermark = 5

    // msg2 begin: write_origin should be past the tail (= 5).
    pb.begin_message(0);
    const float m2[] = {99, 99, 99};
    pb.scatter_add(0, m2, 3);
    pb.commit_source_progress(3);

    std::vector<float> out;
    ASSERT_EQ(drain_exact(pb, out, 8), 8u);
    // [1, 1, 1, 0(tail), 0(tail), 99, 99, 99]
    EXPECT_FLOAT_EQ(out[0], 1.0f);
    EXPECT_FLOAT_EQ(out[1], 1.0f);
    EXPECT_FLOAT_EQ(out[2], 1.0f);
    EXPECT_FLOAT_EQ(out[3], 0.0f);
    EXPECT_FLOAT_EQ(out[4], 0.0f);
    EXPECT_FLOAT_EQ(out[5], 99.0f);
    EXPECT_FLOAT_EQ(out[6], 99.0f);
    EXPECT_FLOAT_EQ(out[7], 99.0f);
}

// ===========================================================================
// SPSC stress test (run under -DOPENCRIEST_TSAN=ON for race detection)
// ===========================================================================

TEST(PairBuffer, ConcurrentProducerConsumerStressNoDataRaces) {
    // Producer: scatter random batches and commit. Consumer: read random
    // batches. Total samples produced == total samples consumed at end.
    constexpr size_t cap = 4096;
    constexpr size_t TOTAL_SAMPLES = 100'000;
    PairBuffer pb(cap, 0);
    pb.begin_message(0);

    std::atomic<size_t> produced{0};
    std::atomic<size_t> consumed{0};

    // Use deterministic per-thread RNG so test is reproducible
    std::thread producer([&] {
        std::mt19937 rng(1234);
        std::uniform_int_distribution<size_t> batch_dist(1, 256);
        size_t pos = 0;
        while (pos < TOTAL_SAMPLES) {
            size_t batch = std::min(batch_dist(rng), TOTAL_SAMPLES - pos);
            // Cap the batch so it fits in the available producer-side window.
            // available_write_approx = capacity - (pos - consumed.load).
            size_t in_flight = pos - consumed.load(std::memory_order_acquire);
            if (in_flight + batch > cap / 2) {
                std::this_thread::sleep_for(std::chrono::microseconds(20));
                continue;
            }
            std::vector<float> in(batch);
            for (size_t i = 0; i < batch; ++i)
                in[i] = static_cast<float>(pos + i);
            pb.scatter_add(pos, in.data(), batch);
            pos += batch;
            pb.commit_source_progress(pos);
            produced.store(pos, std::memory_order_release);
        }
    });

    std::thread consumer([&] {
        std::mt19937 rng(9876);
        std::uniform_int_distribution<size_t> batch_dist(1, 256);
        std::vector<float> out(256);
        size_t expected = 0;
        while (consumed.load(std::memory_order_acquire) < TOTAL_SAMPLES) {
            size_t want = std::min(batch_dist(rng), out.size());
            size_t got = pb.read_advance(out.data(), want);
            if (got == 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
                continue;
            }
            for (size_t i = 0; i < got; ++i) {
                ASSERT_FLOAT_EQ(out[i], static_cast<float>(expected + i))
                    << "expected=" << expected + i << " got=" << out[i];
            }
            expected += got;
            consumed.store(expected, std::memory_order_release);
        }
    });

    producer.join();
    consumer.join();

    EXPECT_EQ(produced.load(), TOTAL_SAMPLES);
    EXPECT_EQ(consumed.load(), TOTAL_SAMPLES);
    EXPECT_EQ(pb.overflow_drops(), 0u);
}

// ===========================================================================
// scatter_add(count == 0) is a no-op
// ===========================================================================

TEST(PairBuffer, ScatterAddZeroCountIsNoOp) {
    PairBuffer pb(4096, 0);
    pb.begin_message(0);
    const float dummy[] = {1.0f};
    pb.scatter_add(0, dummy, 0);
    pb.commit_source_progress(0);
    EXPECT_EQ(pb.available_read(), 0u);
    EXPECT_EQ(pb.overflow_drops(), 0u);
}
