#include "channel/geometric_scene.hpp"
#include "dsp/path_loss.hpp"

#include <algorithm>
#include <cmath>

namespace openCREST {

namespace {

// One candidate path indexed by vertical offset dz: l = sqrt(R^2 + dz^2).
struct Candidate {
    int   order;          // reflection count (bounces)
    float dz;
    float gamma_product;
    bool  has_surface;
    bool  has_bottom;
    bool  enabled;        // individual toggle (orders 0-2); true otherwise
};

float clamp_sound_speed_geom(float c) {
    return (c > 0.0f) ? c : 1500.0f;
}

} // namespace

GeometricScene::GeometricScene(const GeometricSceneConfig& geom,
                               const EnvironmentConfig&    env)
    : geom_(geom), env_(env) {}

std::size_t GeometricScene::compute_paths(float range_m,
                                          std::array<PathTap, MAX_GEOMETRIC_PATHS>& out) const {
    const float zs = geom_.source_depth_m;
    const float zr = geom_.receiver_depth_m;
    const float D  = geom_.water_depth_m;
    const float gs = geom_.gamma_surface;
    const float gb = geom_.gamma_bottom;
    const int   max_b = std::clamp(geom_.max_bounces, 0, kMaxImageOrder);

    // Image-method arrivals grouped by reflection order n. dz is the vertical
    // image offset; l = sqrt(R^2 + dz^2). Orders 0-2 keep their individual
    // enable flags for back-compatibility (the classic five paths); orders
    // 3-4 are gated only by max_bounces. Reflection product carries the
    // surface (gs) and bottom (gb) coefficients once per corresponding bounce.
    const std::array<Candidate, MAX_GEOMETRIC_PATHS> candidates{{
        // order 0 — direct
        { 0, zs - zr,             1.0f,          false, false, geom_.enable_direct },
        // order 1 — one bounce
        { 1, zs + zr,             gs,            true,  false, geom_.enable_surface },
        { 1, 2.0f*D - zs - zr,    gb,            false, true,  geom_.enable_bottom },
        // order 2 — surface-bottom / bottom-surface
        { 2, 2.0f*D - zs + zr,    gs*gb,         true,  true,  geom_.enable_surface_bottom },
        { 2, 2.0f*D + zs - zr,    gs*gb,         true,  true,  geom_.enable_bottom_surface },
        // order 3 — surface-bottom-surface / bottom-surface-bottom
        { 3, 2.0f*D + zs + zr,    gs*gs*gb,      true,  true,  true },
        { 3, 4.0f*D - zs - zr,    gs*gb*gb,      true,  true,  true },
        // order 4 — four bounces
        { 4, 4.0f*D + zs - zr,    gs*gs*gb*gb,   true,  true,  true },
        { 4, 4.0f*D - zs + zr,    gs*gs*gb*gb,   true,  true,  true },
    }};

    std::size_t n = 0;
    for (const auto& c : candidates) {
        if (c.order > max_b || !c.enabled) continue;
        PathTap& t = out[n++];
        t.length_m           = std::sqrt(range_m * range_m + c.dz * c.dz);
        t.reflection_product = c.gamma_product;
        t.has_surface_bounce = c.has_surface;
        t.has_bottom_bounce  = c.has_bottom;
    }

    // Sort by length; direct path (smallest |dz|) comes first when enabled.
    std::sort(out.begin(), out.begin() + n,
              [](const PathTap& a, const PathTap& b) {
                  return a.length_m < b.length_m;
              });
    return n;
}

GeometricScene::ResolvedPath
GeometricScene::resolve(const PathTap& path,
                         float           direct_length_m,
                         float           center_freq_hz,
                         bool            saltwater,
                         float           sample_rate_hz) const {
    const float c_sound = clamp_sound_speed_geom(env_.sound_speed_m_s);

    // Excess delay over direct path, clamped at 0 (direct path itself
    // resolves to 0 samples). Fractional — per-tap Farrow interpolates;
    // static-mode callers round.
    const double excess_s = std::max(0.0,
        (static_cast<double>(path.length_m) - static_cast<double>(direct_length_m))
            / static_cast<double>(c_sound));
    const double delta_samples_frac =
        excess_s * static_cast<double>(sample_rate_hz);

    // Eq.(2), voltage form: a_i = (prod Gamma) / l^(k/2) * 10^(-alpha(fc)*l/20000).
    // k is the power-domain spreading exponent (cylindrical=1, spherical=2,
    // hybrid=1.5) to match static-mode path_loss which uses 10*k*log10(R) dB.
    // Since a_i scales amplitude (voltage), the exponent on l is k/2 — e.g.
    // spherical (k=2) gives 1/R voltage spreading.
    const float fc_khz = center_freq_hz / 1000.0f;
    const float alpha_db_per_km = dsp::thorp_absorption_db_per_km(
        fc_khz, saltwater);
    const float thorp_lin = std::pow(10.0f,
        -(alpha_db_per_km * path.length_m / 1000.0f) / 20.0f);
    const float spread = std::pow(path.length_m,
                                  geom_.spreading_exponent_k * 0.5f);
    const float gain   = path.reflection_product / spread * thorp_lin;

    return ResolvedPath{ delta_samples_frac, gain };
}

} // namespace openCREST
