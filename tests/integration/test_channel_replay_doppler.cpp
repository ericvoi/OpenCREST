// End-to-end replay-mode signal checks: Doppler from a recorded linear
// delay ramp, and destructive interference from a half-cycle delay split
// (the phase-encoded-as-delay convention).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "config/scenario.hpp"
#include "core/types.hpp"
#include "test_helpers/analysis.hpp"
#include "test_helpers/octt_writer.hpp"
#include "test_helpers/signal_generators.hpp"

using openCREST::CalibrationData;
using openCREST::Channel;
using openCREST::ChannelConfig;
using openCREST::ChannelMode;
using openCREST::PairBuffer;

namespace {

constexpr uint32_t FS_HZ   = 500'000;
constexpr float    SOUND   = 1500.0f;
constexpr size_t   BIG_CAP = 1u << 21;

CalibrationData make_cal() {
    CalibrationData c;
    c.adc_sampling_rate = FS_HZ;
    c.dac_sampling_rate = FS_HZ;
    c.adc_bits = 16;
    c.dac_bits = 16;
    c.center_freq_hz = 30'000.0f;
    return c;
}

ChannelConfig make_replay_config(const std::string& trajectory_path) {
    ChannelConfig cfg;
    cfg.from_modem             = "A";
    cfg.to_modem               = "B";
    cfg.range_m                = 3.0f;
    cfg.sound_speed_m_s        = SOUND;
    cfg.saltwater              = true;
    cfg.spreading_factor       = 2.0f;
    cfg.mode                   = ChannelMode::Replay;
    cfg.replay.trajectory_path = trajectory_path;
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

std::vector<float> run_channel(Channel& ch, const std::vector<float>& src) {
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

size_t peak_bin(const std::vector<float>& psd, size_t lo, size_t hi) {
    size_t best = lo;
    for (size_t k = lo; k < std::min(hi, psd.size()); ++k) {
        if (psd[k] > psd[best]) best = k;
    }
    return best;
}

float measure_peak_hz(const std::vector<float>& out, size_t skip,
                      float around_hz) {
    std::vector<float> body(out.begin() + skip, out.begin() + skip + 16384);
    const auto psd = openCREST::test::power_spectrum(body, body.size());
    const float bin_hz = static_cast<float>(FS_HZ) / body.size();
    const size_t k_lo = static_cast<size_t>((around_hz - 1000.0f) / bin_hz);
    const size_t k_hi = static_cast<size_t>((around_hz + 1000.0f) / bin_hz);
    return static_cast<float>(peak_bin(psd, k_lo, k_hi)) * bin_hz;
}

} // namespace

// ===========================================================================
// Constant recorded delay: no shift
// ===========================================================================

TEST(ChannelReplayDoppler, ConstantDelayPreservesFrequency) {
    const std::string path = testing::TempDir() + "flat_delay.octt";
    const uint32_t frames = 41;
    std::vector<double> delays(frames, 0.010);
    std::vector<float>  amps(frames, 1.0f);
    test_helpers::write_octt_file(path, 1, frames, 0.1, 35e3, delays, amps);

    Channel ch(make_replay_config(path), make_cal(), make_cal(), {}, {});

    constexpr float TONE_HZ = 30'000.0f;
    const auto src = openCREST::test::make_tone(
        TONE_HZ, 0.5f, static_cast<float>(FS_HZ), FS_HZ / 2);
    const auto out = run_channel(ch, src);

    const float fp = measure_peak_hz(
        out, ch.base_delay_samples() + 5000 + 256, TONE_HZ);
    const float bin_hz = static_cast<float>(FS_HZ) / 16384.0f;
    EXPECT_NEAR(fp, TONE_HZ, 2.0f * bin_hz);
}

// ===========================================================================
// Linear delay ramp: recorded closing range shifts the tone up
// ===========================================================================

TEST(ChannelReplayDoppler, ShrinkingRecordedDelayShiftsToneUp) {
    // delay(t) = 10 ms - 1 ms/s * t  ->  dtau/dt = -1e-3
    // Received frequency = f0 * (1 - dtau/dt) = f0 * 1.001 -> +30 Hz @ 30 kHz.
    const std::string path = testing::TempDir() + "ramp_delay.octt";
    const uint32_t frames = 41;             // dt = 0.1 s -> 4 s record
    std::vector<double> delays(frames);
    std::vector<float>  amps(frames, 1.0f);
    for (uint32_t i = 0; i < frames; ++i) {
        delays[i] = 0.010 - 0.001 * (0.1 * i);
    }

    test_helpers::write_octt_file(path, 1, frames, 0.1, 35e3, delays, amps);

    auto cfg = make_replay_config(path);
    // Start one frame into the record: the clamped-endpoint Catmull-Rom
    // segment [0, dt] has half the true slope; skip it.
    cfg.replay.offset_s = 0.1;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    constexpr float TONE_HZ = 30'000.0f;
    const size_t N = FS_HZ * 2;              // 2 s tone
    const auto src = openCREST::test::make_tone(
        TONE_HZ, 0.5f, static_cast<float>(FS_HZ), N);
    const auto out = run_channel(ch, src);

    const float fp = measure_peak_hz(
        out, ch.base_delay_samples() + 5000 + 1024, TONE_HZ);
    const float bin_hz = static_cast<float>(FS_HZ) / 16384.0f;
    EXPECT_NEAR(fp, TONE_HZ * 1.001f, 2.0f * bin_hz);
}

// ===========================================================================
// Phase as delay: two equal taps half a tone-cycle apart null out
// ===========================================================================

TEST(ChannelReplayDoppler, HalfCycleDelaySplitCancels) {
    constexpr float TONE_HZ = 30'000.0f;
    // Half a cycle at 30 kHz = 16.667 us; full cycle = 33.333 us.
    const double half_cycle = 0.5 / TONE_HZ;
    const double full_cycle = 1.0 / TONE_HZ;

    auto write_pair = [](const std::string& name, double split) {
        const std::string path = testing::TempDir() + name;
        std::vector<double> delays;
        std::vector<float>  amps;
        for (int f = 0; f < 5; ++f) {
            delays.insert(delays.end(), {0.005, 0.005 + split});
            amps.insert(amps.end(),   {1.0f, 1.0f});
        }
        test_helpers::write_octt_file(path, 2, 5, 0.05, TONE_HZ,
                                      delays, amps);
        return path;
    };

    Channel destructive(make_replay_config(
        write_pair("null_half.octt", half_cycle)),
        make_cal(), make_cal(), {}, {});
    Channel constructive(make_replay_config(
        write_pair("null_full.octt", full_cycle)),
        make_cal(), make_cal(), {}, {});

    const auto src = openCREST::test::make_tone(
        TONE_HZ, 0.5f, static_cast<float>(FS_HZ), 32'768);
    const auto out_null = run_channel(destructive, src);
    const auto out_full = run_channel(constructive, src);

    auto body_rms = [](const std::vector<float>& out, size_t skip) {
        double acc = 0.0;
        const size_t n = 16384;
        for (size_t i = skip; i < skip + n; ++i) acc += out[i] * out[i];
        return std::sqrt(acc / n);
    };
    const size_t skip = destructive.base_delay_samples() + 2600 + 512;
    const double rms_null = body_rms(out_null, skip);
    const double rms_full = body_rms(out_full, skip);

    // Full-cycle split doubles the tone; half-cycle split cancels it.
    EXPECT_LT(rms_null, 0.05 * rms_full);
}
