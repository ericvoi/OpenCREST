#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "channel/channel.hpp"
#include "channel/geometric_scene.hpp"
#include "channel/pair_buffer.hpp"
#include "config/scenario.hpp"
#include "core/types.hpp"
#include "test_helpers/signal_generators.hpp"

using openCREST::CalibrationData;
using openCREST::Channel;
using openCREST::ChannelConfig;
using openCREST::ChannelMode;
using openCREST::EnvironmentConfig;
using openCREST::GeometricScene;
using openCREST::GeometricSceneConfig;
using openCREST::MultipathTap;
using openCREST::PairBuffer;
using openCREST::PathTap;

namespace {

constexpr uint32_t FS         = 500'000;
constexpr float    SOUND      = 1500.0f;
constexpr size_t   BIG_CAP    = 1u << 22;     // 4 Mi samples ≈ 8.4 s @ 500 kSPS
                                              // — must fit base_delay (which
                                              // anchors at r_min, up to ~330k
                                              // samples for the test scenes)
                                              // PLUS the message length.

CalibrationData make_cal(float fc_hz = 25'000.0f) {
    CalibrationData c;
    c.adc_sampling_rate = FS;
    c.dac_sampling_rate = FS;
    c.adc_bits = 16;
    c.dac_bits = 16;
    c.center_freq_hz = fc_hz;
    return c;
}

GeometricSceneConfig base_geom(float r_min = 500.0f, float r_max = 1100.0f) {
    GeometricSceneConfig g;
    g.water_depth_m        = 120.0f;
    g.source_depth_m       =  50.0f;
    g.receiver_depth_m     = 100.0f;
    g.gamma_surface        = -0.9f;
    g.gamma_bottom         =  0.7f;
    g.spreading_exponent_k =  1.5f;
    g.enable_direct        = true;
    g.enable_surface       = true;
    g.enable_bottom        = true;
    g.r_min_m              = r_min;
    g.r_max_m              = r_max;
    return g;
}

ChannelConfig make_geometric_config(GeometricSceneConfig g,
                                     float initial_range_m = 1000.0f,
                                     float velocity_m_s    = 0.0f,
                                     float accel_m_s2      = 0.0f) {
    ChannelConfig cfg;
    cfg.from_modem               = "A";
    cfg.to_modem                 = "B";
    cfg.range_m                  = initial_range_m;  // scenario sizing
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

size_t peak_index(const std::vector<float>& v, size_t from = 0,
                  size_t to = std::numeric_limits<size_t>::max()) {
    to = std::min(to, v.size());
    size_t best = from;
    float best_mag = (from < v.size()) ? std::abs(v[from]) : 0.0f;
    for (size_t i = from + 1; i < to; ++i) {
        if (std::abs(v[i]) > best_mag) { best = i; best_mag = std::abs(v[i]); }
    }
    return best;
}

float hypot_path(float R, float dz) {
    return std::sqrt(R * R + dz * dz);
}

} // namespace

// ===========================================================================
// Construction-time invariants
// ===========================================================================

TEST(ChannelGeometric, BaseDelayAnchoredAtRMin) {
    // direct path length at r_min drives base_delay so excess deltas are
    // always non-negative as R(t) sweeps through [r_min, r_max].
    auto g = base_geom(/*r_min=*/500.0f, /*r_max=*/1100.0f);
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    const float direct_at_r_min = hypot_path(g.r_min_m,
        g.source_depth_m - g.receiver_depth_m);
    const size_t expected = static_cast<size_t>(
        std::round(direct_at_r_min / SOUND * FS));
    EXPECT_EQ(ch.base_delay_samples(), expected);
}

TEST(ChannelGeometric, MaxTapDeltaCoversEnvelope) {
    // The PairBuffer must accommodate the longest enabled reflection at
    // r_max (where slant-path length peaks) minus the direct anchor at
    // r_min. The Channel's reported max_tap_delta must be at least that.
    auto g = base_geom(/*r_min=*/500.0f, /*r_max=*/1100.0f);
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    GeometricScene scene(g, [&](){ EnvironmentConfig e; e.sound_speed_m_s = SOUND; return e; }());
    std::array<PathTap, 5> paths{};
    const size_t n = scene.compute_paths(g.r_max_m, paths);
    ASSERT_GT(n, 1u);
    const float longest_at_r_max = paths[n - 1].length_m;
    const float direct_at_r_min = hypot_path(g.r_min_m,
        g.source_depth_m - g.receiver_depth_m);
    const size_t worst_excess = static_cast<size_t>(
        std::round((longest_at_r_max - direct_at_r_min) / SOUND * FS));

    EXPECT_GE(ch.max_tap_delta_samples(), worst_excess);
}

TEST(ChannelGeometric, StaticModeBaseDelayUnchanged) {
    // Belt-and-suspenders: building a static-mode Channel still produces
    // base_delay = range_m / c (the geometric branch is only entered for
    // mode == Geometric).
    ChannelConfig cfg;
    cfg.from_modem        = "A";
    cfg.to_modem          = "A";
    cfg.range_m           = 3.0f;
    cfg.sound_speed_m_s   = SOUND;
    cfg.saltwater         = true;
    cfg.spreading_factor  = 2.0f;
    cfg.multipath_taps.push_back({0.0f, 1.0f, 0.0f});
    Channel ch(cfg, make_cal(/*fc=*/1.0f), make_cal(/*fc=*/1.0f), {}, {});
    EXPECT_EQ(ch.base_delay_samples(), 1000u);  // 3 m / 1500 m/s · 500 kSPS
}

// ===========================================================================
// on_message_start: scene-supplied taps replace any prior state
// ===========================================================================

TEST(ChannelGeometric, OnMessageStartProducesExpectedNumberOfArrivals) {
    // Direct + surface + bottom enabled → 3 arrivals. Push a single impulse;
    // count peaks above an amplitude threshold within the receiver window.
    auto g = base_geom();
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    auto impulse = openCREST::test::make_impulse(128, 0, 1.0f);
    ch.process(impulse.data(), 128, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();
    ASSERT_GT(out.size(), base + ch.max_tap_delta_samples() + 8);

    // Find peaks above 10% of the largest sample, separated by at least 8
    // samples — a coarse arrival counter.
    float peak_mag = 0.0f;
    for (float v : out) peak_mag = std::max(peak_mag, std::abs(v));
    const float thresh = 0.1f * peak_mag;
    size_t arrivals = 0;
    size_t last_pos = 0;
    for (size_t i = base; i < base + ch.max_tap_delta_samples() + 16; ++i) {
        if (std::abs(out[i]) > thresh && (arrivals == 0 || i - last_pos > 8)) {
            ++arrivals;
            last_pos = i;
        }
    }
    EXPECT_EQ(arrivals, 3u);
}

TEST(ChannelGeometric, DisablingAllReflectionsLeavesDirectOnly) {
    auto g = base_geom();
    g.enable_surface = false;
    g.enable_bottom  = false;
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    auto impulse = openCREST::test::make_impulse(128, 0, 1.0f);
    ch.process(impulse.data(), 128, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();

    float peak_mag = 0.0f;
    for (float v : out) peak_mag = std::max(peak_mag, std::abs(v));
    const float thresh = 0.1f * peak_mag;
    size_t arrivals = 0;
    size_t last_pos = 0;
    for (size_t i = base; i < base + ch.max_tap_delta_samples() + 16; ++i) {
        if (std::abs(out[i]) > thresh && (arrivals == 0 || i - last_pos > 8)) {
            ++arrivals;
            last_pos = i;
        }
    }
    EXPECT_EQ(arrivals, 1u);
}

// ===========================================================================
// Doppler from time-varying delays
// ===========================================================================

TEST(ChannelGeometric, ClosingRangeShortensDirectArrival) {
    // Direct-only scene, closing at 2 m/s. Send an impulse at t=0 and again
    // near the end of a ~1-second message. Δ(arrival_late − arrival_early)
    // should be less than the source-time spacing, by ~v·t·FS/c samples.
    auto g = base_geom(/*r_min=*/990.0f, /*r_max=*/1100.0f);
    g.enable_surface = false;
    g.enable_bottom  = false;
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f,
                                       /*v=*/-2.0f, /*a=*/0.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    // 1 s of source samples with two impulses: at sample 0 and at sample
    // (FS - 1).
    constexpr size_t N = FS;
    std::vector<float> sig(N, 0.0f);
    sig[0]     = 1.0f;
    sig[N - 1] = 1.0f;

    size_t pos = 0;
    while (pos < N) {
        const size_t take = std::min<size_t>(N - pos, 4096);
        const size_t consumed = ch.process(sig.data() + pos, take, pb);
        if (consumed == 0) break;
        pos += consumed;
    }
    ch.on_message_end(pb);

    auto out = drain_all(pb);

    // The early impulse arrives around base + direct_excess(R(0)) + Farrow.
    // Search the first ~half for the earliest peak, second ~half for the
    // latest peak.
    const size_t mid = out.size() / 2;
    const size_t arrival_early = peak_index(out, 0, mid);
    const size_t arrival_late  = peak_index(out, mid, out.size());

    const ptrdiff_t spacing = static_cast<ptrdiff_t>(arrival_late)
                            - static_cast<ptrdiff_t>(arrival_early);
    // Source-time spacing between impulses: N − 1 samples.
    // Doppler-induced compression: −v_closing · t / c · FS ≈ 2 · 1 / 1500 · 5e5
    //                              ≈ 667 samples.
    const ptrdiff_t expected_shift = static_cast<ptrdiff_t>(
        std::round(2.0 * static_cast<double>(N - 1) / SOUND));
    const ptrdiff_t expected_spacing =
        static_cast<ptrdiff_t>(N - 1) - expected_shift;
    EXPECT_NEAR(spacing, expected_spacing, 4)
        << "spacing=" << spacing
        << " expected~=" << expected_spacing;
}

TEST(ChannelGeometric, AccelerationContributesQuadraticTerm) {
    // Zero initial velocity, a = +0.1 m/s² (range increasing). Over 2 s the
    // range grows by 0.5·a·t² = 0.2 m → direct arrival drifts later by
    // 0.2 / 1500 · FS = ~66 samples.
    auto g = base_geom(/*r_min=*/990.0f, /*r_max=*/1010.0f);
    g.enable_surface = false;
    g.enable_bottom  = false;
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f,
                                       /*v=*/0.0f, /*a=*/0.1f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    constexpr size_t N = 2 * FS;
    std::vector<float> sig(N, 0.0f);
    sig[0]     = 1.0f;
    sig[N - 1] = 1.0f;

    size_t pos = 0;
    while (pos < N) {
        const size_t take = std::min<size_t>(N - pos, 4096);
        const size_t consumed = ch.process(sig.data() + pos, take, pb);
        if (consumed == 0) break;
        pos += consumed;
    }
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t mid = out.size() / 2;
    const size_t arrival_early = peak_index(out, 0, mid);
    const size_t arrival_late  = peak_index(out, mid, out.size());

    const ptrdiff_t spacing = static_cast<ptrdiff_t>(arrival_late)
                            - static_cast<ptrdiff_t>(arrival_early);
    // ΔR over 2 s with a=0.1 = 0.2 m. The receiver sees the late impulse
    // ~0.2/c·FS = ~66 samples MORE delayed than the early one, so spacing
    // grows by that amount.
    const ptrdiff_t expected_shift = static_cast<ptrdiff_t>(
        std::round(0.2 / SOUND * FS));
    const ptrdiff_t expected_spacing =
        static_cast<ptrdiff_t>(N - 1) + expected_shift;
    EXPECT_NEAR(spacing, expected_spacing, 4)
        << "spacing=" << spacing
        << " expected~=" << expected_spacing;
}

// ===========================================================================
// Farrow ratio in geometric mode drops the v/c term
// ===========================================================================

TEST(ChannelGeometric, DopplerRatioDropsVelocityTermInGeometricMode) {
    // In geometric mode Doppler emerges from time-varying tap deltas, not
    // from the Farrow ratio — so current_doppler_ratio() should equal
    // (1 + δ_clock) regardless of v.
    auto g = base_geom();
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f,
                                       /*v=*/-5.0f);
    cfg.clock_offset_ppm = 0.0f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    EXPECT_NEAR(ch.current_doppler_ratio(), 1.0, 1e-9);
}

TEST(ChannelGeometric, ClockOffsetStillAppliedInGeometricMode) {
    auto g = base_geom();
    auto cfg = make_geometric_config(g, /*R0=*/1000.0f);
    cfg.clock_offset_ppm = 10.0f;  // 10 ppm
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    EXPECT_NEAR(ch.current_doppler_ratio(), 1.0 + 10e-6, 1e-9);
}
