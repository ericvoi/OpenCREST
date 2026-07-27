#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "channel/replay_tap_source.hpp"
#include "core/tap_trajectory.hpp"
#include "config/scenario.hpp"
#include "test_helpers/octt_writer.hpp"

using openCREST::ReplayConfig;
using openCREST::ReplayTapSource;
using openCREST::SourcedTap;
using openCREST::TapTrajectory;

namespace {

constexpr uint32_t FS = 500'000;

// Two taps, five frames, dt = 50 ms -> duration 0.2 s.
constexpr double kDt       = 0.050;
constexpr double kDuration = 4 * kDt;

std::string write_two_tap_file(const std::string& name) {
    const std::string path = testing::TempDir() + name;
    const std::vector<double> delays = {
        0.0020, 0.0100,
        0.0030, 0.0105,
        0.0025, 0.0110,
        0.0040, 0.0100,
        0.0035, 0.0095,
    };
    const std::vector<float> amps = {
        1.0f, 0.5f,
        0.8f, 0.4f,
        0.9f, 0.3f,
        0.7f, 0.4f,
        0.6f, 0.2f,
    };
    test_helpers::write_octt_file(path, 2, 5, kDt, 35e3, delays, amps);
    return path;
}

ReplayTapSource make_source(const ReplayConfig& cfg,
                            const std::string& name) {
    return ReplayTapSource(TapTrajectory::load(write_two_tap_file(name)),
                           cfg, FS, "A → B");
}

} // namespace

TEST(ReplayTapSource, SizingAccessors) {
    ReplayConfig cfg;
    auto src = make_source(cfg, "sizing.octt");
    EXPECT_EQ(src.tap_count_max(), 2u);
    // ceil(max_delay_s * Fs) = ceil(0.0110 * 500000) = 5500.
    EXPECT_EQ(src.max_tap_delta_samples(),
              static_cast<size_t>(std::ceil(0.0110 * FS)));
}

TEST(ReplayTapSource, TapsAtMatchesTrajectorySamples) {
    const auto path = write_two_tap_file("interp.octt");
    const auto traj = TapTrajectory::load(path);

    ReplayConfig cfg;
    ReplayTapSource src(TapTrajectory::load(path), cfg, FS, "A → B");
    src.on_message_start();

    for (const double t : {0.0, 0.033, 0.101, 0.180}) {
        SourcedTap taps[2];
        ASSERT_EQ(src.taps_at(t, taps, 2), 2u);
        for (size_t k = 0; k < 2; ++k) {
            const auto expected = traj.sample(k, t);
            EXPECT_DOUBLE_EQ(taps[k].delta_samples_frac,
                             expected.delay_s * FS) << "t=" << t;
            EXPECT_FLOAT_EQ(taps[k].gain, expected.amplitude) << "t=" << t;
        }
    }
}

TEST(ReplayTapSource, FixedModeAppliesOffsetAndRestartsEveryMessage) {
    const auto path = write_two_tap_file("fixed.octt");
    const auto traj = TapTrajectory::load(path);

    ReplayConfig cfg;
    cfg.offset_s = 0.060;
    ReplayTapSource src(TapTrajectory::load(path), cfg, FS, "A → B");

    for (int msg = 0; msg < 3; ++msg) {
        src.on_message_start();
        SourcedTap taps[2];
        ASSERT_EQ(src.taps_at(0.020, taps, 2), 2u);
        const auto expected = traj.sample(0, 0.060 + 0.020);
        EXPECT_DOUBLE_EQ(taps[0].delta_samples_frac, expected.delay_s * FS)
            << "message " << msg;
        EXPECT_FLOAT_EQ(taps[0].gain, expected.amplitude) << "message " << msg;
    }
}

TEST(ReplayTapSource, AdvancingModeContinuesByMaxTimeSeen) {
    const auto path = write_two_tap_file("advance.octt");
    const auto traj = TapTrajectory::load(path);

    ReplayConfig cfg;
    cfg.offset_s            = 0.010;
    cfg.advance_per_message = true;
    ReplayTapSource src(TapTrajectory::load(path), cfg, FS, "A → B");

    // Message 1: queries out of order; the high-water mark (0.040),
    // not the last query, must advance the cursor. The 0.040 query stands
    // in for the multipath-tail drain that runs at on_message_end.
    src.on_message_start();
    SourcedTap taps[2];
    (void)src.taps_at(0.020, taps, 2);
    (void)src.taps_at(0.040, taps, 2);
    (void)src.taps_at(0.030, taps, 2);

    // Message 2 starts at 0.010 + 0.040 = 0.050 into the record.
    src.on_message_start();
    ASSERT_EQ(src.taps_at(0.0, taps, 2), 2u);
    const auto expected = traj.sample(0, 0.050);
    EXPECT_DOUBLE_EQ(taps[0].delta_samples_frac, expected.delay_s * FS);
    EXPECT_FLOAT_EQ(taps[0].gain, expected.amplitude);
    EXPECT_DOUBLE_EQ(src.message_offset_s(), 0.050);
}

TEST(ReplayTapSource, AdvancingModeWrapsInsideDeadZone) {
    ReplayConfig cfg;
    cfg.offset_s               = 0.100;
    cfg.advance_per_message    = true;
    cfg.wrap_if_remaining_lt_s = 0.080;   // duration 0.2 s
    auto src = make_source(cfg, "wrap.octt");

    // Message 1 starts at 0.100 (remaining 0.100 >= 0.080 -> no wrap).
    src.on_message_start();
    EXPECT_DOUBLE_EQ(src.message_offset_s(), 0.100);
    SourcedTap taps[2];
    (void)src.taps_at(0.050, taps, 2);

    // Next offset would be 0.150; remaining 0.050 < 0.080 -> wrap to 0.
    src.on_message_start();
    EXPECT_DOUBLE_EQ(src.message_offset_s(), 0.0);
}

TEST(ReplayTapSource, NoWrapWhenDeadZoneDisabled) {
    ReplayConfig cfg;
    cfg.offset_s               = 0.150;
    cfg.advance_per_message    = true;
    cfg.wrap_if_remaining_lt_s = 0.0;
    auto src = make_source(cfg, "no_wrap.octt");

    src.on_message_start();
    EXPECT_DOUBLE_EQ(src.message_offset_s(), 0.150);
    SourcedTap taps[2];
    (void)src.taps_at(0.040, taps, 2);
    src.on_message_start();
    EXPECT_DOUBLE_EQ(src.message_offset_s(), 0.190);
}

TEST(ReplayTapSource, PastEndRampsAmplitudeToZeroWithinOneFrame) {
    const auto path = write_two_tap_file("past_end.octt");
    const auto traj = TapTrajectory::load(path);

    ReplayConfig cfg;
    auto src = make_source(cfg, "past_end2.octt");
    src.on_message_start();

    const auto final_tap0 = traj.sample(0, kDuration);
    SourcedTap taps[2];

    // Exactly at the end: unramped final value.
    ASSERT_EQ(src.taps_at(kDuration, taps, 2), 2u);
    EXPECT_FLOAT_EQ(taps[0].gain, final_tap0.amplitude);

    // Half a frame past the end: half the final amplitude, delay held.
    ASSERT_EQ(src.taps_at(kDuration + 0.5 * kDt, taps, 2), 2u);
    EXPECT_NEAR(taps[0].gain, 0.5f * final_tap0.amplitude, 1e-6f);
    EXPECT_DOUBLE_EQ(taps[0].delta_samples_frac, final_tap0.delay_s * FS);

    // One full frame past the end and beyond: silence, delay held.
    // (NEAR, not EQ: (kDuration + kDt) - kDuration != kDt in doubles.)
    ASSERT_EQ(src.taps_at(kDuration + kDt, taps, 2), 2u);
    EXPECT_NEAR(taps[0].gain, 0.0f, 1e-9f);
    ASSERT_EQ(src.taps_at(kDuration + 10.0, taps, 2), 2u);
    EXPECT_FLOAT_EQ(taps[0].gain, 0.0f);
    EXPECT_DOUBLE_EQ(taps[0].delta_samples_frac, final_tap0.delay_s * FS);
}

TEST(ReplayTapSource, PastEndTimeStillAdvancesTheCursor) {
    // A truncated message's elapsed time (including past-end queries)
    // still counts in advancing mode.
    ReplayConfig cfg;
    cfg.advance_per_message = true;
    auto src = make_source(cfg, "past_end_advance.octt");

    src.on_message_start();
    SourcedTap taps[2];
    (void)src.taps_at(kDuration + 0.050, taps, 2);

    src.on_message_start();
    EXPECT_DOUBLE_EQ(src.message_offset_s(), kDuration + 0.050);
}
