#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "channel/model/geometric/geometric_scene.hpp"
#include "channel/model/geometric/geometric_tap_source.hpp"
#include "config/scenario.hpp"

using openCREST::ChannelConfig;
using openCREST::ChannelMode;
using openCREST::GeometricScene;
using openCREST::GeometricSceneConfig;
using openCREST::GeometricTapSource;
using openCREST::PathTap;
using openCREST::MAX_GEOMETRIC_PATHS;
using openCREST::SourcedTap;

namespace {

constexpr uint32_t FS    = 500'000;
constexpr float    SOUND = 1500.0f;
constexpr float    FC_HZ = 25'000.0f;

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

ChannelConfig make_config(GeometricSceneConfig g,
                          float initial_range_m = 1000.0f,
                          float velocity_m_s    = 0.0f,
                          float accel_m_s2      = 0.0f) {
    ChannelConfig cfg;
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

// Reference tap set computed straight from GeometricScene at range R —
// the values the source must reproduce (model-relative gains; the AFE
// chain multiplies in Channel, not here).
size_t reference_taps(const ChannelConfig& cfg, float range_m,
                      std::array<SourcedTap, 5>& out) {
    openCREST::EnvironmentConfig env;
    env.sound_speed_m_s  = cfg.sound_speed_m_s;
    env.saltwater        = cfg.saltwater;
    env.spreading_factor = cfg.spreading_factor;
    const GeometricScene scene(cfg.geometry, env);

    std::array<PathTap, MAX_GEOMETRIC_PATHS> anchor{};
    const size_t n_anchor = scene.compute_paths(cfg.geometry.r_min_m, anchor);
    EXPECT_GT(n_anchor, 0u);
    const float anchor_len = anchor[0].length_m;

    const float r = std::clamp(range_m,
                               cfg.geometry.r_min_m, cfg.geometry.r_max_m);
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths{};
    const size_t n = scene.compute_paths(r, paths);
    for (size_t i = 0; i < n; ++i) {
        const auto rp = scene.resolve(paths[i], anchor_len, FC_HZ,
                                      cfg.saltwater,
                                      static_cast<float>(FS));
        out[i].delta_samples_frac = rp.delta_samples_frac;
        out[i].gain               = rp.gain_linear;
    }
    return n;
}

} // namespace

TEST(GeometricTapSource, TapsAtMessageStartMatchSceneAtInitialRange) {
    const auto cfg = make_config(base_geom(), 1000.0f);
    GeometricTapSource src(cfg, FC_HZ, FS);
    src.on_message_start();

    std::array<SourcedTap, 5> expected{};
    const size_t n_expected = reference_taps(cfg, 1000.0f, expected);

    std::array<SourcedTap, 5> actual{};
    const size_t n = src.taps_at(0.0, actual.data(), actual.size());
    ASSERT_EQ(n, n_expected);
    for (size_t i = 0; i < n; ++i) {
        EXPECT_DOUBLE_EQ(actual[i].delta_samples_frac,
                         expected[i].delta_samples_frac) << "tap " << i;
        EXPECT_FLOAT_EQ(actual[i].gain, expected[i].gain) << "tap " << i;
    }
}

TEST(GeometricTapSource, TapsFollowRangeUnderVelocityAndAcceleration) {
    const float v = -2.0f;
    const float a = 0.5f;
    const auto cfg = make_config(base_geom(), 1000.0f, v, a);
    GeometricTapSource src(cfg, FC_HZ, FS);
    src.on_message_start();

    for (const double t : {0.5, 2.0, 10.0}) {
        const float r = 1000.0f + v * static_cast<float>(t)
                      + 0.5f * a * static_cast<float>(t * t);
        std::array<SourcedTap, 5> expected{};
        const size_t n_expected = reference_taps(cfg, r, expected);

        std::array<SourcedTap, 5> actual{};
        const size_t n = src.taps_at(t, actual.data(), actual.size());
        ASSERT_EQ(n, n_expected) << "t=" << t;
        for (size_t i = 0; i < n; ++i) {
            EXPECT_DOUBLE_EQ(actual[i].delta_samples_frac,
                             expected[i].delta_samples_frac)
                << "t=" << t << " tap " << i;
            EXPECT_FLOAT_EQ(actual[i].gain, expected[i].gain)
                << "t=" << t << " tap " << i;
        }
    }
}

TEST(GeometricTapSource, ClampsRangeToEnvelope) {
    // v < 0 long enough to drive R(t) below r_min: taps freeze at r_min.
    const auto cfg = make_config(base_geom(500.0f, 1100.0f), 600.0f, -50.0f);
    GeometricTapSource src(cfg, FC_HZ, FS);
    src.on_message_start();

    std::array<SourcedTap, 5> at_rmin{};
    const size_t n_rmin = reference_taps(cfg, 500.0f, at_rmin);

    std::array<SourcedTap, 5> actual{};
    // t = 100 s → unclamped R would be far below r_min.
    const size_t n = src.taps_at(100.0, actual.data(), actual.size());
    ASSERT_EQ(n, n_rmin);
    for (size_t i = 0; i < n; ++i) {
        EXPECT_DOUBLE_EQ(actual[i].delta_samples_frac,
                         at_rmin[i].delta_samples_frac);
        EXPECT_FLOAT_EQ(actual[i].gain, at_rmin[i].gain);
    }
}

TEST(GeometricTapSource, MessageStartResnapshotsInitialRange) {
    const auto cfg = make_config(base_geom(), 1000.0f, -10.0f);
    GeometricTapSource src(cfg, FC_HZ, FS);
    src.on_message_start();

    std::array<SourcedTap, 5> first{};
    const size_t n1 = src.taps_at(5.0, first.data(), first.size());

    // Second message restarts R(t) at initial_range_m — same taps again.
    src.on_message_start();
    std::array<SourcedTap, 5> second{};
    const size_t n2 = src.taps_at(5.0, second.data(), second.size());

    ASSERT_EQ(n1, n2);
    for (size_t i = 0; i < n1; ++i) {
        EXPECT_DOUBLE_EQ(second[i].delta_samples_frac,
                         first[i].delta_samples_frac);
        EXPECT_FLOAT_EQ(second[i].gain, first[i].gain);
    }
}

// Sizing accessors must equal the numbers the Channel constructor derived
// before the refactor (channel.cpp geometric branch).
TEST(GeometricTapSource, SizingAccessorsMatchLegacyFormulas) {
    const auto cfg = make_config(base_geom(500.0f, 1100.0f), 1000.0f);
    GeometricTapSource src(cfg, FC_HZ, FS);

    openCREST::EnvironmentConfig env;
    env.sound_speed_m_s  = cfg.sound_speed_m_s;
    env.saltwater        = cfg.saltwater;
    env.spreading_factor = cfg.spreading_factor;
    const GeometricScene scene(cfg.geometry, env);

    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths_rmin{};
    const size_t n_rmin = scene.compute_paths(cfg.geometry.r_min_m, paths_rmin);
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths_rmax{};
    const size_t n_rmax = scene.compute_paths(cfg.geometry.r_max_m, paths_rmax);
    ASSERT_GT(n_rmin, 0u);
    ASSERT_GT(n_rmax, 0u);

    const float  anchor = paths_rmin[0].length_m;
    const double c      = SOUND;

    const size_t base_delay = static_cast<size_t>(
        std::round(static_cast<double>(anchor) / c * FS));
    EXPECT_EQ(src.base_delay_samples(), base_delay);
    EXPECT_FLOAT_EQ(src.r_min_anchor_len_m(), anchor);

    const float worst_len = paths_rmax[n_rmax - 1].length_m;
    const float worst_excess_s = std::max(
        0.0f, (worst_len - anchor) / static_cast<float>(c));
    EXPECT_EQ(src.max_tap_delta_samples(),
              static_cast<size_t>(std::round(worst_excess_s * FS)));

    const double rmin_excess_s =
        (static_cast<double>(paths_rmin[n_rmin - 1].length_m) - anchor) / c;
    const double rmax_excess_s =
        (static_cast<double>(worst_len) - anchor) / c;
    const size_t sdl_worst = std::max(
        static_cast<size_t>(std::ceil(std::max(0.0, rmin_excess_s) * FS)),
        static_cast<size_t>(std::ceil(std::max(0.0, rmax_excess_s) * FS)));
    EXPECT_EQ(src.sdl_worst_excess_samples(), sdl_worst);

    EXPECT_EQ(src.tap_count_max(), std::max(n_rmin, n_rmax));
}

TEST(GeometricTapSource, ThrowsWhenNoPathEnabled) {
    auto g = base_geom();
    g.enable_direct  = false;
    g.enable_surface = false;
    g.enable_bottom  = false;
    const auto cfg = make_config(g);
    EXPECT_THROW(GeometricTapSource(cfg, FC_HZ, FS), std::invalid_argument);
}

TEST(GeometricTapSource, RespectsCapacityLimit) {
    const auto cfg = make_config(base_geom());
    GeometricTapSource src(cfg, FC_HZ, FS);
    src.on_message_start();

    std::array<SourcedTap, 5> full{};
    const size_t n_full = src.taps_at(0.0, full.data(), full.size());
    ASSERT_GE(n_full, 2u);

    // Undersized output: only `capacity` taps written, count reported as
    // written.
    std::array<SourcedTap, 5> partial{};
    const size_t n_partial = src.taps_at(0.0, partial.data(), 1);
    EXPECT_EQ(n_partial, 1u);
    EXPECT_DOUBLE_EQ(partial[0].delta_samples_frac,
                     full[0].delta_samples_frac);
}
