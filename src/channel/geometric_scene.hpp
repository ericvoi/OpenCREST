#pragma once

#include <array>
#include <cstddef>

#include "config/scenario.hpp"

namespace openCREST {

// One ray-path arrival in the method-of-images scene.
struct PathTap {
    float length_m            = 0.0f;  // total slant range, source → receiver
    float reflection_product  = 1.0f;  // ∏ Γ along the path
    bool  has_surface_bounce  = false;
    bool  has_bottom_bounce   = false;
};

// Method-of-images channel scene: water column with a pressure-release surface
// at z=0 and a partially-reflective bottom at z=water_depth_m. Given a source
// depth, receiver depth, and instantaneous horizontal range R, generates up
// to five enabled path arrivals (direct, surface, bottom, surface→bottom,
// bottom→surface), sorted by length ascending so the direct path (when
// enabled) is always first.
//
// Pure and zero-allocation: compute_paths() / resolve() write into a caller-
// supplied std::array and return scalar PODs, so the Channel hot path can
// call them every block.
class GeometricScene {
public:
    GeometricScene(const GeometricSceneConfig& geom,
                   const EnvironmentConfig&    env);

    // Fill `out` with PathTap entries at horizontal range `range_m`, one per
    // enabled-and-physically-realisable path, sorted by length ascending.
    // Returns the number of entries written (≤ 5).
    std::size_t compute_paths(float range_m,
                              std::array<PathTap, 5>& out) const;

    struct ResolvedPath {
        // Excess over the direct path expressed in fractional receiver-rate
        // samples. Session C consumes this directly via the per-tap Farrow's
        // read-style interpolator; legacy static-mode call sites round to
        // size_t at the call boundary.
        double delta_samples_frac;
        float  gain_linear;         // eq.(2): Γ-product / ℓ^k · 10^(-α·ℓ/20000)
    };

    // Convert a single PathTap to its sample-domain (delay, amplitude) pair
    // given the supplied direct-path length (so reflections come out with
    // excess delay ≥ 0). center_freq_hz drives Thorp absorption.
    ResolvedPath resolve(const PathTap& path,
                         float           direct_length_m,
                         float           center_freq_hz,
                         bool            saltwater,
                         float           sample_rate_hz) const;

    const GeometricSceneConfig& config()      const { return geom_; }
    const EnvironmentConfig&    environment() const { return env_;  }

private:
    GeometricSceneConfig geom_;
    EnvironmentConfig    env_;
};

} // namespace openCREST
