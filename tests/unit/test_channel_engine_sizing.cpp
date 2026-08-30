#include <gtest/gtest.h>

#include "channel/channel_engine.hpp"
#include "config/scenario.hpp"
#include "core/constants.hpp"

using namespace openCREST;

namespace {

constexpr uint32_t kFs = 500'000;

ScenarioConfig make_scenario(float max_msg_dur_s) {
    ScenarioConfig sc;
    sc.name = "sizing_test";
    sc.environment.sound_speed_m_s        = 1500.0f;
    sc.environment.max_message_duration_s = max_msg_dur_s;
    return sc;
}

} // namespace

// pair_capacity_for_extent takes the worst per-channel write extent — which
// every propagation model reports the same way via
// Channel::write_extent_samples() — and adds the environment floor plus the
// in-flight message slack. It holds no model knowledge, so these tests need
// no channels at all.

TEST(ChannelEngineSizing, IncludesMaxMessageDuration) {
    const auto sc = make_scenario(/*max_msg_dur_s=*/3.0f);

    // 150 m at 1500 m/s = 50_000 samples of base delay, no multipath.
    const size_t extent = 50'000;
    const size_t cap = ChannelEngine::pair_capacity_for_extent(extent, sc, kFs);

    const size_t in_flight = static_cast<size_t>(3.0f * kFs);
    EXPECT_EQ(cap, extent + in_flight);
}

TEST(ChannelEngineSizing, ScalesWithLongerMessages) {
    const size_t extent = 50'000;

    const size_t cap_1s = ChannelEngine::pair_capacity_for_extent(
        extent, make_scenario(1.0f), kFs);
    const size_t cap_10s = ChannelEngine::pair_capacity_for_extent(
        extent, make_scenario(10.0f), kFs);

    // 9 s of additional in-flight budget at 500 kSPS = 4.5M extra samples.
    EXPECT_EQ(cap_10s, cap_1s + 9 * kFs);
}

TEST(ChannelEngineSizing, ZeroOrNegativeFallsBackToTenSeconds) {
    auto sc = make_scenario(0.0f);
    sc.environment.max_message_duration_s = 0.0f;  // unset

    const size_t extent = 50'000;
    const size_t cap = ChannelEngine::pair_capacity_for_extent(extent, sc, kFs);

    const size_t in_flight = static_cast<size_t>(10.0f * kFs);
    EXPECT_EQ(cap, extent + in_flight);
}

// A longer per-channel extent must widen the buffer one-for-one, whatever
// model produced it.
TEST(ChannelEngineSizing, TracksWorstChannelExtent) {
    const auto sc = make_scenario(3.0f);

    const size_t small = ChannelEngine::pair_capacity_for_extent(
        50'000, sc, kFs);
    const size_t large = ChannelEngine::pair_capacity_for_extent(
        50'000 + 64'000, sc, kFs);

    EXPECT_EQ(large, small + 64'000);
}

// environment.max_range_m declares a worst case that may exceed any single
// channel's configured extent; it raises the floor but never lowers it.
TEST(ChannelEngineSizing, EnvironmentMaxRangeRaisesFloor) {
    auto sc = make_scenario(3.0f);
    sc.environment.max_range_m = 1500.0f;   // 1 s of propagation

    const size_t cap = ChannelEngine::pair_capacity_for_extent(
        /*worst_channel_extent=*/1'000, sc, kFs);

    const size_t env_base  = static_cast<size_t>(1500.0f * kFs / 1500.0f);
    const size_t env_mp    = static_cast<size_t>(MAX_MULTIPATH_DELAY_S * kFs);
    const size_t in_flight = static_cast<size_t>(3.0f * kFs);
    EXPECT_EQ(cap, env_base + env_mp + in_flight);
}

TEST(ChannelEngineSizing, EnvironmentMaxRangeDoesNotShrinkALargerExtent) {
    auto sc = make_scenario(3.0f);
    sc.environment.max_range_m = 15.0f;   // far smaller than the channel

    const size_t extent = 400'000;
    const size_t cap = ChannelEngine::pair_capacity_for_extent(extent, sc, kFs);

    EXPECT_EQ(cap, extent + static_cast<size_t>(3.0f * kFs));
}

// An empty scenario must still yield a usable buffer rather than zero.
TEST(ChannelEngineSizing, ZeroExtentFallsBackToOneSecondFloor) {
    const auto sc = make_scenario(3.0f);

    const size_t cap = ChannelEngine::pair_capacity_for_extent(0, sc, kFs);

    EXPECT_EQ(cap, static_cast<size_t>(kFs) + static_cast<size_t>(3.0f * kFs));
}

// Regression: a 3 s message at 150 m loopback must exceed the 2^20 sample
// ceiling (2.097 s) so the message tail isn't silently truncated.
TEST(ChannelEngineSizing, RegressionLoopbackTruncationAt2_1Seconds) {
    const auto sc = make_scenario(/*max_msg_dur_s=*/3.0f);

    const size_t cap = ChannelEngine::pair_capacity_for_extent(
        /*worst_channel_extent=*/50'000, sc, kFs);
    EXPECT_GT(cap, static_cast<size_t>(1'048'576))
        << "Pair buffer capacity must exceed the 2^20 ceiling.";
}
