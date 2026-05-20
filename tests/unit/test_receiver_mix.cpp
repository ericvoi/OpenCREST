#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <numeric>

#include "channel/receiver_mix.hpp"
#include "channel/pair_buffer.hpp"
#include "core/ring_buffer.hpp"
#include "core/sample_conversion.hpp"
#include "core/types.hpp"
#include "dsp/noise_generator.hpp"

using openCREST::CalibrationData;
using openCREST::PairBuffer;
using openCREST::ReceiverMix;
using openCREST::SPSCRingBuffer;
using openCREST::dsp::NoiseConfig;

namespace {

CalibrationData make_cal() {
    CalibrationData c;
    c.adc_bits = 16;
    c.dac_bits = 16;
    c.adc_sampling_rate = 500'000;
    c.dac_sampling_rate = 500'000;
    return c;
}

NoiseConfig silent_noise() {
    NoiseConfig n;
    n.wenz_sea_state        = 0;
    n.target_level_db_re_fs = -200.0f;  // effectively zero
    n.saltwater             = true;
    return n;
}

constexpr uint16_t MID = 1u << 15;  // 16-bit midpoint

} // namespace

// ===========================================================================
// Empty incoming + silent noise: rx_ring fills with midpoint samples
// ===========================================================================

TEST(ReceiverMix, NoIncomingProducesSilenceAtMidpoint) {
    SPSCRingBuffer<uint16_t> rx_ring(1024);

    ReceiverMix mix({}, silent_noise(), make_cal(), 500'000u);

    EXPECT_EQ(mix.pull(rx_ring, 64), 64u);

    std::vector<uint16_t> out(64);
    EXPECT_EQ(rx_ring.read(out.data(), 64), 64u);

    for (size_t i = 0; i < 64; ++i) {
        // Output should be very close to midpoint (silence)
        const int dev = std::abs(static_cast<int>(out[i]) - static_cast<int>(MID));
        EXPECT_LT(dev, 200) << "i=" << i << " v=" << out[i];
    }
}

// ===========================================================================
// Single incoming: PairBuffer data is mixed → DAC → rx_ring
// ===========================================================================

TEST(ReceiverMix, SingleIncomingPropagatesToRxRing) {
    PairBuffer pb(1024, /*base_delay=*/0);
    pb.begin_message(0);
    std::vector<float> data(64, 0.5f);  // half-amplitude DC
    pb.scatter_add(0, data.data(), 64);
    pb.commit_source_progress(64);

    SPSCRingBuffer<uint16_t> rx_ring(1024);
    ReceiverMix mix({&pb}, silent_noise(), make_cal(), 500'000u);

    EXPECT_EQ(mix.pull(rx_ring, 64), 64u);
    // PairBuffer should have been drained.
    EXPECT_EQ(pb.available_read(), 0u);

    std::vector<uint16_t> out(64);
    EXPECT_EQ(rx_ring.read(out.data(), 64), 64u);

    // 0.5 in float maps to ≈ 0.75 of full scale (mid + 0.5*mid)
    const uint16_t expected = openCREST::float_to_dac(0.5f, 16);
    for (size_t i = 0; i < 64; ++i) {
        EXPECT_NEAR(static_cast<int>(out[i]),
                    static_cast<int>(expected), 200)
            << "i=" << i;
    }
}

// ===========================================================================
// Multiple incoming PairBuffers sum
// ===========================================================================

TEST(ReceiverMix, MultipleIncomingSum) {
    PairBuffer pb1(1024, 0);
    PairBuffer pb2(1024, 0);

    pb1.begin_message(0);
    pb2.begin_message(0);

    std::vector<float> a(32, 0.2f);
    std::vector<float> b(32, 0.3f);
    pb1.scatter_add(0, a.data(), 32);
    pb2.scatter_add(0, b.data(), 32);
    pb1.commit_source_progress(32);
    pb2.commit_source_progress(32);

    SPSCRingBuffer<uint16_t> rx_ring(1024);
    ReceiverMix mix({&pb1, &pb2}, silent_noise(), make_cal(), 500'000u);

    ASSERT_EQ(mix.pull(rx_ring, 32), 32u);

    std::vector<uint16_t> out(32);
    rx_ring.read(out.data(), 32);

    const uint16_t expected = openCREST::float_to_dac(0.5f, 16);  // 0.2 + 0.3
    for (size_t i = 0; i < 32; ++i) {
        EXPECT_NEAR(static_cast<int>(out[i]),
                    static_cast<int>(expected), 200);
    }
}

// ===========================================================================
// PairBuffer empty positions read as 0 (PairBuffer guarantees zeros there)
// ===========================================================================

TEST(ReceiverMix, EmptyPairBufferContributesZero) {
    PairBuffer pb(1024, 8);  // base_delay=8 → first 8 samples are silence
    pb.begin_message(0);
    std::vector<float> data(8, 1.0f);
    pb.scatter_add(0, data.data(), 8);
    pb.commit_source_progress(8);

    SPSCRingBuffer<uint16_t> rx_ring(1024);
    ReceiverMix mix({&pb}, silent_noise(), make_cal(), 500'000u);

    ASSERT_EQ(mix.pull(rx_ring, 16), 16u);

    std::vector<uint16_t> out(16);
    rx_ring.read(out.data(), 16);

    // First 8 positions are propagation-delay zeros → midpoint DAC.
    for (size_t i = 0; i < 8; ++i) {
        const int dev = std::abs(static_cast<int>(out[i]) - static_cast<int>(MID));
        EXPECT_LT(dev, 200) << "i=" << i;
    }
    // Last 8 are 1.0 → max DAC.
    const uint16_t expected = openCREST::float_to_dac(1.0f, 16);
    for (size_t i = 8; i < 16; ++i) {
        EXPECT_NEAR(static_cast<int>(out[i]),
                    static_cast<int>(expected), 200) << "i=" << i;
    }
}

// ===========================================================================
// PairBuffer always advances even when rx_ring is full (the drop is the
// visible loss; receiver clock keeps up with wall time).
// ===========================================================================

TEST(ReceiverMix, AdvancesPairBuffersOnRxRingOverflow) {
    PairBuffer pb(1024, 0);
    pb.begin_message(0);
    std::vector<float> data(128, 0.5f);
    pb.scatter_add(0, data.data(), 128);
    pb.commit_source_progress(128);

    // rx_ring intentionally tiny so writes get clipped.
    SPSCRingBuffer<uint16_t> rx_ring(8);
    ReceiverMix mix({&pb}, silent_noise(), make_cal(), 500'000u);

    const size_t accepted = mix.pull(rx_ring, 128);
    // Some samples accepted, but PairBuffer fully drained.
    EXPECT_LT(accepted, 128u);
    EXPECT_EQ(pb.available_read(), 0u);
    EXPECT_GT(mix.rx_ring_drops(), 0u);
}

// ===========================================================================
// Clipping: very loud sum is clamped, not wrapped
// ===========================================================================

TEST(ReceiverMix, OverloadIsClampedNotWrapped) {
    PairBuffer pb(1024, 0);
    pb.begin_message(0);
    std::vector<float> data(16, 5.0f);  // way past full-scale
    pb.scatter_add(0, data.data(), 16);
    pb.commit_source_progress(16);

    SPSCRingBuffer<uint16_t> rx_ring(1024);
    ReceiverMix mix({&pb}, silent_noise(), make_cal(), 500'000u);

    ASSERT_EQ(mix.pull(rx_ring, 16), 16u);

    std::vector<uint16_t> out(16);
    rx_ring.read(out.data(), 16);
    const uint16_t max16 = (1u << 16) - 1;
    for (size_t i = 0; i < 16; ++i) {
        EXPECT_EQ(out[i], max16) << "i=" << i;
    }
}

// ===========================================================================
// pull(0) is a safe no-op
// ===========================================================================

TEST(ReceiverMix, PullZeroIsNoOp) {
    SPSCRingBuffer<uint16_t> rx_ring(1024);
    ReceiverMix mix({}, silent_noise(), make_cal(), 500'000u);
    EXPECT_EQ(mix.pull(rx_ring, 0), 0u);
    EXPECT_EQ(rx_ring.available_read(), 0u);
}
