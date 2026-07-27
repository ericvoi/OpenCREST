#include <gtest/gtest.h>

#include <cmath>

#include "channel/channel_engine.hpp"
#include "config/scenario.hpp"
#include "core/constants.hpp"

using namespace openCREST;

namespace {

ScenarioConfig make_loopback(float range_m, float max_msg_dur_s) {
    ScenarioConfig sc;
    sc.name = "sizing_test";
    sc.environment.sound_speed_m_s        = 1500.0f;
    sc.environment.max_message_duration_s = max_msg_dur_s;

    ModemConfig mc;
    mc.id         = "modem_a";
    mc.usb_serial = "SN-A";
    sc.modems.push_back(mc);

    ChannelConfig cc;
    cc.from_modem = "modem_a";
    cc.to_modem   = "modem_a";
    cc.range_m    = range_m;
    cc.multipath_taps.push_back({0.0f, 1.0f, 0.0f});
    sc.channels.push_back(cc);
    return sc;
}

} // namespace

// worst_case_pair_capacity must include the configured message duration.

TEST(ChannelEngineSizing, IncludesMaxMessageDuration) {
    constexpr uint32_t kFs = 500'000;
    const auto sc = make_loopback(/*range_m=*/150.0f, /*max_msg_dur_s=*/3.0f);

    const size_t cap = ChannelEngine::worst_case_pair_capacity(sc, kFs);

    // base_delay = 50_000 (100 ms), multipath = 100_000 (200 ms),
    // in_flight = 1_500_000 (3.0 s) → at least 1_650_000 samples.
    const size_t base_delay = static_cast<size_t>(150.0f * kFs / 1500.0f);
    const size_t multipath  = static_cast<size_t>(MAX_MULTIPATH_DELAY_S * kFs);
    const size_t in_flight  = static_cast<size_t>(3.0f * kFs);
    EXPECT_GE(cap, base_delay + multipath + in_flight);
}

TEST(ChannelEngineSizing, ScalesWithLongerMessages) {
    constexpr uint32_t kFs = 500'000;

    const size_t cap_1s = ChannelEngine::worst_case_pair_capacity(
        make_loopback(150.0f, 1.0f), kFs);
    const size_t cap_10s = ChannelEngine::worst_case_pair_capacity(
        make_loopback(150.0f, 10.0f), kFs);

    // 9 s of additional in-flight budget at 500 kSPS = 4.5M extra samples.
    EXPECT_GE(cap_10s, cap_1s + 9 * kFs);
}

TEST(ChannelEngineSizing, ZeroOrNegativeFallsBackToTenSeconds) {
    constexpr uint32_t kFs = 500'000;
    auto sc = make_loopback(150.0f, 0.0f);
    sc.environment.max_message_duration_s = 0.0f;  // unset

    const size_t cap = ChannelEngine::worst_case_pair_capacity(sc, kFs);

    // Fallback applies → capacity must accommodate >= 10 s of in-flight.
    const size_t base_delay = static_cast<size_t>(150.0f * kFs / 1500.0f);
    const size_t multipath  = static_cast<size_t>(MAX_MULTIPATH_DELAY_S * kFs);
    const size_t in_flight  = static_cast<size_t>(10.0f * kFs);
    EXPECT_GE(cap, base_delay + multipath + in_flight);
}

// A 3 s message at 150 m loopback must exceed the 2^20 sample ceiling
// (2.097 s) so the message tail isn't silently truncated.
TEST(ChannelEngineSizing, RegressionLoopbackTruncationAt2_1Seconds) {
    constexpr uint32_t kFs = 500'000;
    const auto sc = make_loopback(/*range_m=*/150.0f, /*max_msg_dur_s=*/3.0f);

    const size_t cap = ChannelEngine::worst_case_pair_capacity(sc, kFs);
    EXPECT_GT(cap, static_cast<size_t>(1'048'576))
        << "Pair buffer capacity must exceed the 2^20 ceiling.";
}

// Replay channels: extent = base_delay (range or propagation_delay_s
// override) + the trajectory's max excess delay (loader-populated).

TEST(ChannelEngineSizing, ReplayExtentUsesTrajectoryMaxDelay) {
    constexpr uint32_t kFs = 500'000;
    auto sc = make_loopback(/*range_m=*/150.0f, /*max_msg_dur_s=*/3.0f);
    sc.channels[0].multipath_taps.clear();
    sc.channels[0].mode               = ChannelMode::Replay;
    sc.channels[0].range_m            = 800.0f;
    sc.channels[0].replay.max_delay_s = 0.128;

    const size_t cap = ChannelEngine::worst_case_pair_capacity(sc, kFs);

    const size_t base      = static_cast<size_t>(800.0f * kFs / 1500.0f);
    const size_t mp        = static_cast<size_t>(std::ceil(0.128 * kFs));
    const size_t in_flight = static_cast<size_t>(3.0f * kFs);
    EXPECT_GE(cap, base + mp + in_flight);
}

TEST(ChannelEngineSizing, ReplayExtentHonorsPropagationDelayOverride) {
    constexpr uint32_t kFs = 500'000;
    auto sc = make_loopback(150.0f, 3.0f);
    sc.channels[0].multipath_taps.clear();
    sc.channels[0].mode                = ChannelMode::Replay;
    sc.channels[0].range_m             = 150.0f;
    sc.channels[0].propagation_delay_s = 2.0f;   // >> range-derived delay
    sc.channels[0].replay.max_delay_s  = 0.050;

    const size_t cap = ChannelEngine::worst_case_pair_capacity(sc, kFs);

    const size_t base      = static_cast<size_t>(2.0f * kFs);
    const size_t mp        = static_cast<size_t>(std::ceil(0.050 * kFs));
    const size_t in_flight = static_cast<size_t>(3.0f * kFs);
    EXPECT_GE(cap, base + mp + in_flight);
}
