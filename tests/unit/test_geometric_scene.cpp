#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "channel/geometric_scene.hpp"
#include "config/scenario.hpp"
#include "dsp/path_loss.hpp"

using openCREST::EnvironmentConfig;
using openCREST::GeometricScene;
using openCREST::GeometricSceneConfig;
using openCREST::PathTap;
using openCREST::MAX_GEOMETRIC_PATHS;

namespace {

constexpr float FS         = 500'000.0f;
constexpr float SOUND_M_S  = 1500.0f;
constexpr float FC_HZ      = 25'000.0f;

GeometricSceneConfig base_geom() {
    GeometricSceneConfig g;
    g.water_depth_m         = 120.0f;
    g.source_depth_m        =  50.0f;
    g.receiver_depth_m      = 100.0f;
    g.gamma_surface         = -0.9f;
    g.gamma_bottom          =  0.7f;
    g.spreading_exponent_k  =  1.5f;
    g.enable_direct         = true;
    g.enable_surface        = true;
    g.enable_bottom         = true;
    g.enable_surface_bottom = false;
    g.enable_bottom_surface = false;
    return g;
}

EnvironmentConfig base_env() {
    EnvironmentConfig e;
    e.sound_speed_m_s = SOUND_M_S;
    e.saltwater       = true;
    return e;
}

float hypot_path(float R, float dz) {
    return std::sqrt(R * R + dz * dz);
}

} // namespace

// ---------------------------------------------------------------------------
// Closed-form path lengths
// ---------------------------------------------------------------------------

TEST(GeometricScene, DirectPathClosedForm) {
    auto g = base_geom();
    g.enable_surface = false;
    g.enable_bottom  = false;
    GeometricScene scene(g, base_env());

    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(1000.0f, out);
    ASSERT_EQ(n, 1u);
    EXPECT_NEAR(out[0].length_m,
                hypot_path(1000.0f, g.source_depth_m - g.receiver_depth_m),
                1e-2f);
    EXPECT_FLOAT_EQ(out[0].reflection_product, 1.0f);
    EXPECT_FALSE(out[0].has_surface_bounce);
    EXPECT_FALSE(out[0].has_bottom_bounce);
}

TEST(GeometricScene, SurfacePathClosedForm) {
    auto g = base_geom();
    g.enable_direct = false;
    g.enable_bottom = false;
    GeometricScene scene(g, base_env());

    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(1000.0f, out);
    ASSERT_EQ(n, 1u);
    EXPECT_NEAR(out[0].length_m,
                hypot_path(1000.0f, g.source_depth_m + g.receiver_depth_m),
                1e-2f);
    EXPECT_FLOAT_EQ(out[0].reflection_product, g.gamma_surface);
    EXPECT_TRUE (out[0].has_surface_bounce);
    EXPECT_FALSE(out[0].has_bottom_bounce);
}

TEST(GeometricScene, BottomPathClosedForm) {
    auto g = base_geom();
    g.enable_direct  = false;
    g.enable_surface = false;
    GeometricScene scene(g, base_env());

    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(1000.0f, out);
    ASSERT_EQ(n, 1u);
    const float dz = (g.water_depth_m - g.source_depth_m)
                   + (g.water_depth_m - g.receiver_depth_m);
    EXPECT_NEAR(out[0].length_m, hypot_path(1000.0f, dz), 1e-2f);
    EXPECT_FLOAT_EQ(out[0].reflection_product, g.gamma_bottom);
    EXPECT_FALSE(out[0].has_surface_bounce);
    EXPECT_TRUE (out[0].has_bottom_bounce);
}

TEST(GeometricScene, SurfaceBottomClosedForm) {
    auto g = base_geom();
    g.enable_direct         = false;
    g.enable_surface        = false;
    g.enable_bottom         = false;
    g.enable_surface_bottom = true;
    GeometricScene scene(g, base_env());

    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(800.0f, out);
    ASSERT_EQ(n, 1u);
    const float dz = (2.0f * g.water_depth_m - g.source_depth_m) + g.receiver_depth_m;
    EXPECT_NEAR(out[0].length_m, hypot_path(800.0f, dz), 1e-2f);
    EXPECT_FLOAT_EQ(out[0].reflection_product,
                    g.gamma_surface * g.gamma_bottom);
    EXPECT_TRUE(out[0].has_surface_bounce);
    EXPECT_TRUE(out[0].has_bottom_bounce);
}

TEST(GeometricScene, BottomSurfaceClosedForm) {
    auto g = base_geom();
    g.enable_direct         = false;
    g.enable_surface        = false;
    g.enable_bottom         = false;
    g.enable_bottom_surface = true;
    GeometricScene scene(g, base_env());

    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(800.0f, out);
    ASSERT_EQ(n, 1u);
    const float dz = g.source_depth_m + (2.0f * g.water_depth_m - g.receiver_depth_m);
    EXPECT_NEAR(out[0].length_m, hypot_path(800.0f, dz), 1e-2f);
    EXPECT_FLOAT_EQ(out[0].reflection_product,
                    g.gamma_bottom * g.gamma_surface);
    EXPECT_TRUE(out[0].has_surface_bounce);
    EXPECT_TRUE(out[0].has_bottom_bounce);
}

// ---------------------------------------------------------------------------
// Path ordering, enable/disable counts
// ---------------------------------------------------------------------------

TEST(GeometricScene, PathsOrderedByLengthAscending) {
    auto g = base_geom();
    g.enable_surface_bottom = true;
    g.enable_bottom_surface = true;
    GeometricScene scene(g, base_env());

    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(1000.0f, out);
    ASSERT_EQ(n, 5u);
    for (size_t i = 1; i < n; ++i) {
        EXPECT_LE(out[i - 1].length_m, out[i].length_m)
            << "paths must be sorted ascending; failed at i=" << i;
    }
}

TEST(GeometricScene, DirectPathFirstWhenEnabled) {
    GeometricScene scene(base_geom(), base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(1000.0f, out);
    ASSERT_GE(n, 1u);
    EXPECT_FLOAT_EQ(out[0].reflection_product, 1.0f);  // direct
}

TEST(GeometricScene, DisablingPathsReducesCount) {
    auto g = base_geom();
    g.enable_surface = false;
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    EXPECT_EQ(scene.compute_paths(1000.0f, out), 2u);   // direct + bottom

    g.enable_bottom = false;
    GeometricScene scene2(g, base_env());
    EXPECT_EQ(scene2.compute_paths(1000.0f, out), 1u);  // direct only
}

TEST(GeometricScene, AllPathsDisabledReturnsZero) {
    auto g = base_geom();
    g.enable_direct  = false;
    g.enable_surface = false;
    g.enable_bottom  = false;
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    EXPECT_EQ(scene.compute_paths(1000.0f, out), 0u);
}

// ---------------------------------------------------------------------------
// resolve(): tap delta = excess over direct; gain = eq.(2)
// ---------------------------------------------------------------------------

TEST(GeometricScene, ResolveDirectDeltaIsZero) {
    GeometricScene scene(base_geom(), base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths{};
    const size_t n = scene.compute_paths(1000.0f, paths);
    ASSERT_GE(n, 1u);
    const auto resolved = scene.resolve(paths[0], paths[0].length_m,
                                         FC_HZ, /*saltwater=*/true, FS);
    EXPECT_DOUBLE_EQ(resolved.delta_samples_frac, 0.0);
}

TEST(GeometricScene, ResolveReflectionDeltaIsPositive) {
    GeometricScene scene(base_geom(), base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths{};
    const size_t n = scene.compute_paths(1000.0f, paths);
    ASSERT_GE(n, 2u);
    const float direct_len = paths[0].length_m;
    for (size_t i = 1; i < n; ++i) {
        const auto r = scene.resolve(paths[i], direct_len, FC_HZ, true, FS);
        EXPECT_GT(r.delta_samples_frac, 0.0)
            << "reflected path " << i << " must have non-zero excess delay";
    }
}

TEST(GeometricScene, ResolveDeltaMatchesExcessOverDirectInSamples) {
    GeometricScene scene(base_geom(), base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths{};
    const size_t n = scene.compute_paths(1000.0f, paths);
    ASSERT_GE(n, 2u);
    const float direct_len = paths[0].length_m;
    for (size_t i = 1; i < n; ++i) {
        const auto r = scene.resolve(paths[i], direct_len, FC_HZ, true, FS);
        const double expected_excess_s =
            (static_cast<double>(paths[i].length_m) -
             static_cast<double>(direct_len)) / SOUND_M_S;
        const double expected_samples_frac = expected_excess_s * FS;
        EXPECT_NEAR(r.delta_samples_frac, expected_samples_frac, 1e-6)
            << "path " << i;
    }
}

TEST(GeometricScene, ResolveDeltaIsFractional) {
    // Geometry chosen so the path length yields a non-integer sample delay.
    auto g = base_geom();
    g.enable_surface = false;
    g.enable_bottom  = false;
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths{};
    // Range 1234.0 m with depth difference -50 m → length ~1235.012 m, direct
    // anchor = length itself, so resolve against a slightly shorter anchor
    // to exercise fractional excess.
    const size_t n = scene.compute_paths(1234.0f, paths);
    ASSERT_EQ(n, 1u);
    const float fake_anchor = 1234.0f;  // shorter than length_m
    const auto r = scene.resolve(paths[0], fake_anchor, FC_HZ, true, FS);
    // Excess must be non-integer (i.e., the rounding is gone).
    const double rounded = std::round(r.delta_samples_frac);
    EXPECT_NE(r.delta_samples_frac, rounded);
}

TEST(GeometricScene, ResolveGainMatchesEq2HandComputation) {
    // Hand-computed eq.(2): a = (∏ Γ) / ℓ^k · 10^(-α(fc)·ℓ/20000)
    auto g = base_geom();
    g.spreading_exponent_k = 1.5f;
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths{};
    const size_t n = scene.compute_paths(1000.0f, paths);
    ASSERT_GE(n, 2u);

    const float fc_khz = FC_HZ / 1000.0f;
    const float alpha  = openCREST::dsp::thorp_absorption_db_per_km(
        fc_khz, /*saltwater=*/true);

    for (size_t i = 0; i < n; ++i) {
        const auto r = scene.resolve(paths[i], paths[0].length_m,
                                      FC_HZ, true, FS);
        const float ell = paths[i].length_m;
        const float thorp_lin = std::pow(10.0f,
            -(alpha * ell / 1000.0f) / 20.0f);
        const float expected =
            paths[i].reflection_product
            / std::pow(ell, g.spreading_exponent_k * 0.5f)
            * thorp_lin;
        EXPECT_NEAR(r.gain_linear, expected, std::abs(expected) * 1e-3f + 1e-12f)
            << "path " << i << " (length " << ell << ")";
    }
}

// ---------------------------------------------------------------------------
// Higher-order image expansion (max_bounces)
// ---------------------------------------------------------------------------

namespace {
// Locate the path whose slant length matches the closed-form dz at range R.
const PathTap* path_with_dz(const std::array<PathTap, MAX_GEOMETRIC_PATHS>& out,
                            size_t n, float R, float dz) {
    const float want = hypot_path(R, dz);
    for (size_t i = 0; i < n; ++i)
        if (std::abs(out[i].length_m - want) < 1e-1f) return &out[i];
    return nullptr;
}
} // namespace

TEST(GeometricScene, MaxBouncesDefaultsToFivePaths) {
    // Full low-order scene, default max_bounces (2) -> the classic five paths.
    auto g = base_geom();
    g.enable_surface_bottom = true;
    g.enable_bottom_surface = true;
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    EXPECT_EQ(scene.compute_paths(1000.0f, out), 5u);
}

TEST(GeometricScene, MaxBouncesRaisesPathCount) {
    auto g = base_geom();
    g.enable_surface_bottom = true;
    g.enable_bottom_surface = true;
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};

    g.max_bounces = 3;
    EXPECT_EQ(GeometricScene(g, base_env()).compute_paths(1000.0f, out), 7u);

    g.max_bounces = 4;
    EXPECT_EQ(GeometricScene(g, base_env()).compute_paths(1000.0f, out), 9u);
}

TEST(GeometricScene, OrderThreePathsClosedForm) {
    auto g = base_geom();          // D=120 zs=50 zr=100 gs=-0.9 gb=0.7
    g.enable_surface_bottom = true;
    g.enable_bottom_surface = true;
    g.max_bounces = 3;
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(1000.0f, out);

    // surface-bottom-surface: dz = 2D + zs + zr, Gamma = gs^2 * gb.
    const PathTap* sbs = path_with_dz(out, n, 1000.0f, 2*120.0f + 50.0f + 100.0f);
    ASSERT_NE(sbs, nullptr);
    EXPECT_NEAR(sbs->reflection_product, 0.81f * 0.7f, 1e-5f);
    EXPECT_TRUE(sbs->has_surface_bounce);
    EXPECT_TRUE(sbs->has_bottom_bounce);

    // bottom-surface-bottom: dz = 4D - zs - zr, Gamma = gs * gb^2.
    const PathTap* bsb = path_with_dz(out, n, 1000.0f, 4*120.0f - 50.0f - 100.0f);
    ASSERT_NE(bsb, nullptr);
    EXPECT_NEAR(bsb->reflection_product, -0.9f * 0.49f, 1e-5f);
}

TEST(GeometricScene, OrderFourPathsClosedForm) {
    auto g = base_geom();
    g.enable_surface_bottom = true;
    g.enable_bottom_surface = true;
    g.max_bounces = 4;
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    const size_t n = scene.compute_paths(1000.0f, out);

    // Both order-4 paths carry Gamma = gs^2 * gb^2.
    const PathTap* p1 = path_with_dz(out, n, 1000.0f, 4*120.0f + 50.0f - 100.0f);
    const PathTap* p2 = path_with_dz(out, n, 1000.0f, 4*120.0f - 50.0f + 100.0f);
    ASSERT_NE(p1, nullptr);
    ASSERT_NE(p2, nullptr);
    EXPECT_NEAR(p1->reflection_product, 0.81f * 0.49f, 1e-5f);
    EXPECT_NEAR(p2->reflection_product, 0.81f * 0.49f, 1e-5f);
}

TEST(GeometricScene, HigherOrderRespectsDirectToggleButNotLowOrderFlags) {
    // Orders 3-4 are gated only by max_bounces, independent of the order-2
    // enable flags; the low-order flags still gate their own paths.
    auto g = base_geom();
    g.enable_surface_bottom = false;   // order-2 paths off ...
    g.enable_bottom_surface = false;
    g.max_bounces = 4;                 // ... but order 3-4 on
    GeometricScene scene(g, base_env());
    std::array<PathTap, MAX_GEOMETRIC_PATHS> out{};
    // direct + surface + bottom (3) + order3 (2) + order4 (2) = 7.
    EXPECT_EQ(scene.compute_paths(1000.0f, out), 7u);
}
