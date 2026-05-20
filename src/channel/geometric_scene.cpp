#include "channel/geometric_scene.hpp"
#include "dsp/path_loss.hpp"

#include <algorithm>
#include <cmath>

namespace openCREST {

namespace {

// One candidate path indexed by its vertical offset Δz: ℓ = √(R² + Δz²).
struct Candidate {
    float dz;
    float gamma_product;
    bool  has_surface;
    bool  has_bottom;
    bool  enabled;
};

float clamp_sound_speed_geom(float c) {
    return (c > 0.0f) ? c : 1500.0f;
}

} // namespace

GeometricScene::GeometricScene(const GeometricSceneConfig& geom,
                               const EnvironmentConfig&    env)
    : geom_(geom), env_(env) {}

std::size_t GeometricScene::compute_paths(float range_m,
                                          std::array<PathTap, 5>& out) const {
    const float zs = geom_.source_depth_m;
    const float zr = geom_.receiver_depth_m;
    const float D  = geom_.water_depth_m;

    // Paper eq.(1) — image-method Δz for each path.
    const std::array<Candidate, 5> candidates{{
        { zs - zr,                       1.0f,                                   false, false, geom_.enable_direct },
        { zs + zr,                       geom_.gamma_surface,                    true,  false, geom_.enable_surface },
        { (D - zs) + (D - zr),           geom_.gamma_bottom,                     false, true,  geom_.enable_bottom },
        { (2.0f * D - zs) + zr,          geom_.gamma_surface * geom_.gamma_bottom, true, true,  geom_.enable_surface_bottom },
        { zs + (2.0f * D - zr),          geom_.gamma_bottom * geom_.gamma_surface, true, true,  geom_.enable_bottom_surface },
    }};

    std::size_t n = 0;
    for (const auto& c : candidates) {
        if (!c.enabled) continue;
        PathTap& t = out[n++];
        t.length_m           = std::sqrt(range_m * range_m + c.dz * c.dz);
        t.reflection_product = c.gamma_product;
        t.has_surface_bounce = c.has_surface;
        t.has_bottom_bounce  = c.has_bottom;
    }

    // Sort by length ascending — direct (smallest |Δz|) naturally comes
    // first whenever it's enabled.
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

    // Excess delay over direct path (clamped at 0; the direct path itself
    // resolves to 0 samples by construction). Fractional — Session C reads
    // this with per-tap Farrow interpolation; legacy callers round.
    const double excess_s = std::max(0.0,
        (static_cast<double>(path.length_m) - static_cast<double>(direct_length_m))
            / static_cast<double>(c_sound));
    const double delta_samples_frac =
        excess_s * static_cast<double>(sample_rate_hz);

    // Eq.(2), voltage form: a_i = (∏ Γ) / ℓ^(k/2) · 10^(-α(fc)·ℓ/20000).
    // The k convention is the power-domain spreading exponent (cylindrical=1,
    // spherical=2, hybrid=1.5) to match static-mode path_loss, which computes
    // 10·k·log10(R) dB. Since a_i scales sample amplitude (voltage), the
    // exponent on ℓ is k/2 — e.g. spherical (k=2) gives 1/R voltage spreading,
    // matching the static path-loss reduction factor 10^(-20·log10(R)/20).
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
