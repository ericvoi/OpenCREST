#pragma once

#include <cstdint>

#include "channel/geometric_scene.hpp"
#include "channel/tap_source.hpp"
#include "config/scenario.hpp"

namespace openCREST {

// Analytic tap source: method-of-images scene evaluated at
// R(t) = R_0 + v*t + 0.5*a*t^2, clamped to [r_min, r_max]. R_0 is
// re-snapshotted at every message start (initial_range_m, falling back to
// range_m). Emits model-relative gains (spreading + reflection + Thorp);
// the Channel multiplies the AFE-electrical chain on top.
class GeometricTapSource final : public TapSource {
public:
    // Throws std::invalid_argument when the scene has no enabled path at
    // r_min or r_max.
    GeometricTapSource(const ChannelConfig& config,
                       float                source_center_fc_hz,
                       uint32_t             sample_rate);

    void   on_message_start() override;
    size_t taps_at(double t_seconds, SourcedTap* out,
                   size_t capacity) override;
    size_t tap_count_max()         const override { return tap_count_; }
    size_t max_tap_delta_samples() const override {
        return max_tap_delta_samples_;
    }

    // Non-virtual sizing accessors for the Channel constructor.

    // Base propagation delay anchored at the shortest enabled path at
    // r_min, so every tap's excess delta stays >= 0 across the R envelope.
    size_t base_delay_samples()      const { return base_delay_samples_; }
    float  r_min_anchor_len_m()      const { return r_min_anchor_len_m_; }
    // Worst read-back excess for SourceDelayLine sizing (ceil; checked at
    // both r_min and r_max — strong reflections can put it at r_min).
    size_t sdl_worst_excess_samples() const {
        return sdl_worst_excess_samples_;
    }

private:
    float range_at_source_time(double t_seconds) const;

    GeometricScene scene_;

    // Scene-evaluation parameters (copied from ChannelConfig at init).
    float    cfg_initial_range_m_;
    float    range_m_;
    float    velocity_m_s_;
    float    acceleration_m_s2_;
    float    r_min_m_;
    float    r_max_m_;
    float    center_fc_hz_;
    bool     saltwater_;
    uint32_t sample_rate_;

    // R_0 snapshot, refreshed each on_message_start().
    float    initial_range_m_ = 0.0f;

    // Init-time sizing.
    float    r_min_anchor_len_m_       = 0.0f;
    size_t   tap_count_                = 0;
    size_t   base_delay_samples_       = 0;
    size_t   max_tap_delta_samples_    = 0;
    size_t   sdl_worst_excess_samples_ = 0;
};

} // namespace openCREST
