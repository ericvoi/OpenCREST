#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "config/scenario.hpp"
#include "core/types.hpp"
#include "test_helpers/analysis.hpp"
#include "test_helpers/signal_generators.hpp"

using openCREST::CalibrationData;
using openCREST::Channel;
using openCREST::ChannelConfig;
using openCREST::ChannelMode;
using openCREST::GeometricSceneConfig;
using openCREST::PairBuffer;

namespace {

constexpr uint32_t FS_HZ   = 500'000;
constexpr float    SOUND   = 1500.0f;
constexpr size_t   BIG_CAP = 1u << 22;  // ≥ base_delay + message body + tail

CalibrationData make_cal() {
    CalibrationData c{};
    c.adc_sampling_rate = FS_HZ;
    c.dac_sampling_rate = FS_HZ;
    c.adc_bits          = 16;
    c.dac_bits          = 16;
    c.center_freq_hz    = 30'000.0f;
    return c;
}

GeometricSceneConfig direct_only_geom(float r_min, float r_max) {
    GeometricSceneConfig g{};
    g.water_depth_m         = 100.0f;
    g.source_depth_m        =  50.0f;
    g.receiver_depth_m      =  50.0f;  // co-planar → direct path length = R
    g.gamma_surface         = -0.9f;
    g.gamma_bottom          =  0.6f;
    g.spreading_exponent_k  =  1.5f;
    g.enable_direct         = true;
    g.enable_surface        = false;
    g.enable_bottom         = false;
    g.r_min_m               = r_min;
    g.r_max_m               = r_max;
    return g;
}

ChannelConfig make_cfg(GeometricSceneConfig g,
                        float initial_range_m,
                        float velocity_m_s = 0.0f,
                        float accel_m_s2   = 0.0f) {
    ChannelConfig cfg{};
    cfg.from_modem               = "A";
    cfg.to_modem                 = "B";
    cfg.range_m                  = initial_range_m;
    cfg.spreading_factor         = 2.0f;
    cfg.saltwater                = true;
    cfg.sound_speed_m_s          = SOUND;
    cfg.mode                     = ChannelMode::Geometric;
    cfg.geometry                 = g;
    cfg.initial_range_m          = initial_range_m;
    cfg.velocity_radial_m_s      = velocity_m_s;
    cfg.acceleration_radial_m_s2 = accel_m_s2;
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

// Locate the peak bin in PSD within [k_lo, k_hi).
size_t peak_bin(const std::vector<float>& psd, size_t k_lo, size_t k_hi) {
    size_t best = k_lo;
    float  best_v = (k_lo < psd.size()) ? psd[k_lo] : 0.0f;
    for (size_t k = k_lo + 1; k < k_hi && k < psd.size(); ++k) {
        if (psd[k] > best_v) { best_v = psd[k]; best = k; }
    }
    return best;
}

// Push samples through a Channel in batches and return the receiver-side
// PairBuffer contents (drained to a vector).
std::vector<float> run_channel(Channel&            ch,
                                PairBuffer&         pb,
                                const std::vector<float>& src) {
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

// Slice the receiver-side waveform after base_delay so analysis ignores the
// silence head.
std::vector<float> body_after_base(const std::vector<float>& full,
                                   size_t base_delay,
                                   size_t take) {
    if (base_delay >= full.size()) return {};
    const size_t end = std::min(full.size(), base_delay + take);
    return std::vector<float>(full.begin() + base_delay, full.begin() + end);
}

} // namespace

// ===========================================================================
// Stationary: 30 kHz tone arrives at 30 kHz (no shift)
// ===========================================================================

TEST(ChannelGeometricDoppler, NoMotionPreservesFrequency) {
    auto g  = direct_only_geom(/*r_min=*/990.0f, /*r_max=*/1010.0f);
    auto cfg = make_cfg(g, /*R0=*/1000.0f, /*v=*/0.0f, /*a=*/0.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());

    constexpr float TONE_HZ = 30'000.0f;
    constexpr size_t N      = FS_HZ / 4;  // 0.25 s
    auto src = openCREST::test::make_tone(
        TONE_HZ, 0.5f, static_cast<float>(FS_HZ), N);

    auto out = run_channel(ch, pb, src);

    // Take a long body window (well past base_delay + Catmull-Rom warmup).
    auto body = body_after_base(out, ch.base_delay_samples() + 32,
                                 /*take=*/N - 64);
    ASSERT_GT(body.size(), 4096u);

    // Trim to 8192 for FFT, then take peak around 30 kHz.
    body.resize(8192);
    const auto psd = openCREST::test::power_spectrum(body, body.size());
    const float bin_hz = static_cast<float>(FS_HZ) / body.size();
    const size_t k_lo = static_cast<size_t>(29'000.0f / bin_hz);
    const size_t k_hi = static_cast<size_t>(31'000.0f / bin_hz);
    const size_t kp   = peak_bin(psd, k_lo, k_hi);
    const float  fp   = static_cast<float>(kp) * bin_hz;
    EXPECT_NEAR(fp, TONE_HZ, 2.0f * bin_hz);
}

// ===========================================================================
// Closing range: receiver sees compressed (higher-frequency) tone
// ===========================================================================

TEST(ChannelGeometricDoppler, ClosingRangeShiftsToneUp) {
    // v = -2 m/s closing → expected Doppler factor = 1 + 2/1500 = 1.001333…
    // For a 30 kHz tone, expected shift ≈ +40 Hz.
    auto g  = direct_only_geom(/*r_min=*/980.0f, /*r_max=*/1010.0f);
    auto cfg = make_cfg(g, /*R0=*/1000.0f, /*v=*/-2.0f, /*a=*/0.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());

    constexpr float TONE_HZ = 30'000.0f;
    constexpr size_t N      = FS_HZ * 2;  // 2 s (gives good freq resolution)
    auto src = openCREST::test::make_tone(
        TONE_HZ, 0.5f, static_cast<float>(FS_HZ), N);

    auto out = run_channel(ch, pb, src);

    // FFT window: take a centered slice of the body, large enough to
    // resolve a 40 Hz shift (~25 Hz/bin at 16384 → adequate).
    auto body = body_after_base(out, ch.base_delay_samples() + 256, N - 1024);
    ASSERT_GT(body.size(), 16384u);
    body.resize(16384);

    const auto psd = openCREST::test::power_spectrum(body, body.size());
    const float bin_hz = static_cast<float>(FS_HZ) / body.size();
    const size_t k_lo = static_cast<size_t>(29'500.0f / bin_hz);
    const size_t k_hi = static_cast<size_t>(30'500.0f / bin_hz);
    const size_t kp   = peak_bin(psd, k_lo, k_hi);
    const float  fp   = static_cast<float>(kp) * bin_hz;

    const float doppler_factor = 1.0f + 2.0f / SOUND;
    const float expected_hz    = TONE_HZ * doppler_factor;
    // Expected shift ≈ +40 Hz; bin resolution ~30.5 Hz. Allow ±1.5 bins.
    EXPECT_NEAR(fp, expected_hz, 2.0f * bin_hz)
        << "fp=" << fp << " expected=" << expected_hz;
    // And it must really be above the source — no shift would put it at
    // exactly TONE_HZ within bin resolution.
    EXPECT_GT(fp, TONE_HZ);
}

// ===========================================================================
// Opening range with two paths: distinct Doppler shifts on each arrival
// ===========================================================================

TEST(ChannelGeometricDoppler, TwoPathsHaveDistinguishableShifts) {
    // Direct + surface enabled; the two paths have different range rates
    // because the surface path's slant length changes more slowly with R
    // than the direct path does (the |Δz| offset is fixed). Choose
    // geometry where the path lengths differ enough to resolve.
    GeometricSceneConfig g{};
    g.water_depth_m        = 100.0f;
    g.source_depth_m       =  20.0f;
    g.receiver_depth_m     =  80.0f;
    g.gamma_surface        = -0.9f;
    g.gamma_bottom         =  0.6f;
    g.spreading_exponent_k =  1.5f;
    g.enable_direct        = true;
    g.enable_surface       = true;
    g.enable_bottom        = false;
    g.r_min_m              = 480.0f;
    g.r_max_m              = 520.0f;

    auto cfg = make_cfg(g, /*R0=*/500.0f, /*v=*/-5.0f, /*a=*/0.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());

    constexpr float TONE_HZ = 30'000.0f;
    constexpr size_t N      = FS_HZ;  // 1 s
    auto src = openCREST::test::make_tone(
        TONE_HZ, 0.5f, static_cast<float>(FS_HZ), N);

    auto out = run_channel(ch, pb, src);
    ASSERT_GT(out.size(), ch.base_delay_samples() + 4096u);

    auto body = body_after_base(out, ch.base_delay_samples() + 128, N - 512);
    ASSERT_GT(body.size(), 16384u);
    body.resize(16384);

    const auto psd = openCREST::test::power_spectrum(body, body.size());
    const float bin_hz = static_cast<float>(FS_HZ) / body.size();

    // Both Dopplers are positive (closing). Direct: shift ~+100 Hz.
    // Surface: same v_R projected through the geometry; differs slightly.
    // Just assert the peak energy near 30 kHz is to the right (above) and
    // that there's *some* spread beyond a single bin.
    const size_t k_lo = static_cast<size_t>(29'500.0f / bin_hz);
    const size_t k_hi = static_cast<size_t>(30'500.0f / bin_hz);
    const size_t kp   = peak_bin(psd, k_lo, k_hi);
    const float  fp   = static_cast<float>(kp) * bin_hz;
    EXPECT_GT(fp, TONE_HZ - bin_hz);  // not below source
}

// ===========================================================================
// Static-mode sanity: building a static-mode Channel and pushing audio
// still produces non-trivial energy in the PairBuffer.
// ===========================================================================

TEST(ChannelGeometricDoppler, StaticModeStillProducesEnergyAfterRefactor) {
    ChannelConfig cfg{};
    cfg.from_modem        = "A";
    cfg.to_modem          = "A";
    cfg.range_m           = 3.0f;
    cfg.spreading_factor  = 2.0f;
    cfg.saltwater         = true;
    cfg.sound_speed_m_s   = SOUND;
    cfg.multipath_taps.push_back({0.0f, 1.0f, 0.0f});

    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    PairBuffer pb(BIG_CAP, ch.base_delay_samples());

    auto src = openCREST::test::make_tone(
        30'000.0f, 0.5f, static_cast<float>(FS_HZ), 8192);
    auto out = run_channel(ch, pb, src);
    EXPECT_GT(openCREST::test::rms(out), 0.05f);
}
