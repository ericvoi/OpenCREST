#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "channel/model/replay/tap_trajectory.hpp"
#include "config/scenario.hpp"
#include "core/types.hpp"
#include "test_helpers/octt_writer.hpp"
#include "test_helpers/signal_generators.hpp"

using openCREST::CalibrationData;
using openCREST::Channel;
using openCREST::ChannelConfig;
using openCREST::ChannelMode;
using openCREST::PairBuffer;
using openCREST::TapTrajectoryError;

namespace {

constexpr uint32_t FS      = 500'000;
constexpr float    SOUND   = 1500.0f;
constexpr size_t   BIG_CAP = 1u << 21;

CalibrationData make_cal() {
    CalibrationData c;
    c.adc_sampling_rate = FS;
    c.dac_sampling_rate = FS;
    c.adc_bits = 16;
    c.dac_bits = 16;
    c.center_freq_hz = 30'000.0f;
    return c;
}

// Two constant taps: 1 ms @ 1.0 and 10 ms @ 0.5, five frames over 0.2 s.
std::string write_constant_two_tap(const std::string& name) {
    const std::string path = testing::TempDir() + name;
    std::vector<double> delays;
    std::vector<float>  amps;
    for (int f = 0; f < 5; ++f) {
        delays.insert(delays.end(), {0.001, 0.010});
        amps.insert(amps.end(),   {1.0f, 0.5f});
    }
    test_helpers::write_octt_file(path, 2, 5, 0.050, 35e3, delays, amps);
    return path;
}

ChannelConfig make_replay_config(const std::string& trajectory_path,
                                 float range_m = 3.0f) {
    ChannelConfig cfg;
    cfg.from_modem              = "A";
    cfg.to_modem                = "B";
    cfg.range_m                 = range_m;
    cfg.sound_speed_m_s         = SOUND;
    cfg.saltwater               = true;
    cfg.spreading_factor        = 2.0f;
    cfg.mode                    = ChannelMode::Replay;
    cfg.replay.trajectory_path  = trajectory_path;
    return cfg;
}

std::vector<float> drain_all(PairBuffer& pb) {
    std::vector<float> out;
    std::vector<float> tmp(4096);
    while (true) {
        const size_t n = pb.read_advance(tmp.data(), tmp.size());
        if (n == 0) break;
        out.insert(out.end(), tmp.begin(), tmp.begin() + n);
    }
    return out;
}

std::vector<float> run_message(Channel& ch, const std::vector<float>& src) {
    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);
    size_t pos = 0;
    while (pos < src.size()) {
        const size_t take = std::min<size_t>(src.size() - pos, 4096);
        const size_t consumed = ch.process(src.data() + pos, take, pb);
        if (consumed == 0) break;
        pos += consumed;
    }
    ch.on_message_end(pb);
    return drain_all(pb);
}

size_t peak_index(const std::vector<float>& v, size_t from, size_t to) {
    to = std::min(to, v.size());
    size_t best = from;
    float best_mag = 0.0f;
    for (size_t i = from; i < to; ++i) {
        if (std::abs(v[i]) > best_mag) { best = i; best_mag = std::abs(v[i]); }
    }
    return best;
}

} // namespace

// ===========================================================================
// Construction-time invariants
// ===========================================================================

TEST(ChannelReplay, BaseDelayFromRange) {
    const auto cfg = make_replay_config(
        write_constant_two_tap("base_delay.octt"), /*range_m=*/3.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    EXPECT_EQ(ch.base_delay_samples(), 1000u);   // 3 m / 1500 m/s · 500 kSPS
}

TEST(ChannelReplay, PropagationDelayOverridesRange) {
    auto cfg = make_replay_config(
        write_constant_two_tap("prop_delay.octt"), 3.0f);
    cfg.propagation_delay_s = 0.010f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    EXPECT_EQ(ch.base_delay_samples(), 5000u);
}

TEST(ChannelReplay, MaxTapDeltaFromTrajectory) {
    const auto cfg = make_replay_config(
        write_constant_two_tap("max_delta.octt"));
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    EXPECT_EQ(ch.max_tap_delta_samples(),
              static_cast<size_t>(std::ceil(0.010 * FS)));
}

TEST(ChannelReplay, ThrowsOnMissingTrajectoryFile) {
    const auto cfg = make_replay_config("/nonexistent/nope.octt");
    EXPECT_THROW(Channel(cfg, make_cal(), make_cal(), {}, {}),
                 TapTrajectoryError);
}

// ===========================================================================
// Rendering
// ===========================================================================

TEST(ChannelReplay, ImpulseArrivesAtTrajectoryDelays) {
    const auto cfg = make_replay_config(
        write_constant_two_tap("impulse.octt"));
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    auto impulse = openCREST::test::make_impulse(512, 8, 1.0f);
    const auto out = run_message(ch, impulse);

    const size_t base = ch.base_delay_samples();
    ASSERT_GE(out.size(), base + 5000 + 512);

    // Tap 1: excess 0.001 s = 500 samples; tap 2: 0.010 s = 5000 samples.
    const size_t p1 = peak_index(out, base, base + 2000);
    const size_t p2 = peak_index(out, base + 4000, base + 6000);
    EXPECT_EQ(p1, base + 8 + 500);
    EXPECT_EQ(p2, base + 8 + 5000);

    // Relative amplitude 0.5 (absolute scale is the AFE chain gain).
    EXPECT_NEAR(std::abs(out[p2]) / std::abs(out[p1]), 0.5f, 0.01f);
}

TEST(ChannelReplay, FixedModeMessagesAreIdentical) {
    const auto cfg = make_replay_config(
        write_constant_two_tap("repeat.octt"));
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    const auto tone = openCREST::test::make_tone(
        30'000.0f, 0.5f, static_cast<float>(FS), 4096);

    const auto first  = run_message(ch, tone);
    const auto second = run_message(ch, tone);

    ASSERT_EQ(first.size(), second.size());
    for (size_t i = 0; i < first.size(); ++i) {
        ASSERT_FLOAT_EQ(first[i], second[i]) << "sample " << i;
    }
}

TEST(ChannelReplay, DeadTapContributesSilence) {
    // Single tap, amplitude zero across the whole record.
    const std::string path = testing::TempDir() + "dead.octt";
    std::vector<double> delays(5, 0.002);
    std::vector<float>  amps(5, 0.0f);
    test_helpers::write_octt_file(path, 1, 5, 0.050, 35e3, delays, amps);

    const auto cfg = make_replay_config(path);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    const auto tone = openCREST::test::make_tone(
        30'000.0f, 0.5f, static_cast<float>(FS), 4096);
    const auto out = run_message(ch, tone);

    for (const float v : out) ASSERT_EQ(v, 0.0f);
}

TEST(ChannelReplay, FadingTapAmplitudeFollowsTrajectory) {
    // Single constant-delay tap fading 1.0 -> 0.0 across a 0.2 s record;
    // a 0.15 s tone must come out much louder at the head than the tail.
    const std::string path = testing::TempDir() + "fade.octt";
    std::vector<double> delays;
    std::vector<float>  amps;
    for (int f = 0; f < 5; ++f) {
        delays.push_back(0.002);
        amps.push_back(1.0f - 0.25f * static_cast<float>(f));
    }
    test_helpers::write_octt_file(path, 1, 5, 0.050, 35e3, delays, amps);

    const auto cfg = make_replay_config(path);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    const size_t n = FS * 3 / 20;   // 0.15 s
    const auto tone = openCREST::test::make_tone(
        30'000.0f, 0.5f, static_cast<float>(FS), n);
    const auto out = run_message(ch, tone);

    const size_t body = ch.base_delay_samples() + 1000 + 2048;
    ASSERT_GT(out.size(), body + n - 8192);

    auto rms_of = [&](size_t from, size_t count) {
        double acc = 0.0;
        for (size_t i = from; i < from + count; ++i) acc += out[i] * out[i];
        return std::sqrt(acc / count);
    };
    const double head = rms_of(body, 8192);
    // Tail slice ends inside the tone (0.14 s in -> amplitude ~0.3).
    const double tail = rms_of(body + n - 16384, 8192);
    EXPECT_GT(head, 2.5 * tail);
    EXPECT_GT(tail, 0.0);
}

TEST(ChannelReplay, TailDrainCoversLongestTap) {
    // The multipath tail past the last input sample must still carry the
    // 10 ms tap's decaying contribution.
    const auto cfg = make_replay_config(
        write_constant_two_tap("tail.octt"));
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    auto impulse = openCREST::test::make_impulse(64, 32, 1.0f);
    const auto out = run_message(ch, impulse);

    const size_t base = ch.base_delay_samples();
    // The second tap's arrival sits 5000 samples after the input ended —
    // only reachable if on_message_end drained the tail.
    ASSERT_GT(out.size(), base + 32 + 5000);
    EXPECT_GT(std::abs(out[base + 32 + 5000]), 0.0f);
}
