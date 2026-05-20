#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <numeric>
#include "core/ring_buffer.hpp"

using openCREST::SPSCRingBuffer;

// ---------------------------------------------------------------------------
// Basic single-thread correctness
// ---------------------------------------------------------------------------

TEST(SPSCRingBuffer, CapacityRoundsUpToPowerOfTwo) {
    SPSCRingBuffer<int> rb(100);
    // 100 → next power of two = 128
    EXPECT_EQ(rb.capacity(), 128u);
}

TEST(SPSCRingBuffer, EmptyOnConstruction) {
    SPSCRingBuffer<float> rb(64);
    EXPECT_EQ(rb.available_read(), 0u);
    EXPECT_EQ(rb.available_write(), 64u);
}

TEST(SPSCRingBuffer, WriteAndReadRoundTrip) {
    SPSCRingBuffer<int> rb(16);
    int src[] = {1, 2, 3, 4, 5};
    EXPECT_EQ(rb.write(src, 5), 5u);
    EXPECT_EQ(rb.available_read(), 5u);

    int dst[5] = {};
    EXPECT_EQ(rb.read(dst, 5), 5u);
    for (int i = 0; i < 5; ++i) EXPECT_EQ(dst[i], src[i]);
    EXPECT_EQ(rb.available_read(), 0u);
}

TEST(SPSCRingBuffer, FullBufferBlocksWrite) {
    SPSCRingBuffer<int> rb(4);
    int data[] = {1, 2, 3, 4, 5, 6};
    // rb capacity is 4; writing 6 should only write 4
    EXPECT_EQ(rb.write(data, 6), 4u);
    EXPECT_EQ(rb.available_read(), 4u);
    EXPECT_EQ(rb.available_write(), 0u);
}

TEST(SPSCRingBuffer, EmptyBufferBlocksRead) {
    SPSCRingBuffer<int> rb(8);
    int dst[4];
    EXPECT_EQ(rb.read(dst, 4), 0u);
}

TEST(SPSCRingBuffer, WrapAround) {
    SPSCRingBuffer<int> rb(4);  // capacity = 4 (already power of 2)
    int data[] = {10, 20, 30, 40};
    rb.write(data, 4);

    int out[4];
    rb.read(out, 2); // consume 10, 20

    int more[] = {50, 60};
    rb.write(more, 2); // write past the end, wraps

    rb.read(out, 4); // should get 30, 40, 50, 60
    EXPECT_EQ(out[0], 30);
    EXPECT_EQ(out[1], 40);
    EXPECT_EQ(out[2], 50);
    EXPECT_EQ(out[3], 60);
}

TEST(SPSCRingBuffer, Reset) {
    SPSCRingBuffer<int> rb(8);
    int data[] = {1, 2, 3};
    rb.write(data, 3);
    rb.reset();
    EXPECT_EQ(rb.available_read(), 0u);
    EXPECT_EQ(rb.available_write(), 8u);
}

// ---------------------------------------------------------------------------
// Concurrent producer / consumer stress test
// ---------------------------------------------------------------------------

TEST(SPSCRingBuffer, ConcurrentStress) {
    constexpr size_t N = 100'000;
    SPSCRingBuffer<uint32_t> rb(1024);

    std::vector<uint32_t> received;
    received.reserve(N);

    std::thread producer([&] {
        for (uint32_t i = 0; i < N; ) {
            uint32_t batch[64];
            size_t count = std::min<size_t>(64, N - i);
            for (size_t j = 0; j < count; ++j) batch[j] = i + j;
            size_t written = rb.write(batch, count);
            i += written;
            // Yield if full to avoid busy-spinning indefinitely in CI
            if (written == 0) std::this_thread::yield();
        }
    });

    std::thread consumer([&] {
        while (received.size() < N) {
            uint32_t buf[64];
            size_t n = rb.read(buf, 64);
            for (size_t i = 0; i < n; ++i) received.push_back(buf[i]);
            if (n == 0) std::this_thread::yield();
        }
    });

    producer.join();
    consumer.join();

    ASSERT_EQ(received.size(), N);
    for (size_t i = 0; i < N; ++i) {
        EXPECT_EQ(received[i], static_cast<uint32_t>(i)) << "mismatch at " << i;
        if (received[i] != static_cast<uint32_t>(i)) break;
    }
}
