#include "channel/geometric_tap_source.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace openCREST {

namespace {

double clamp_sound_speed(float configured) {
    return (configured > 0.0f) ? static_cast<double>(configured) : 1500.0;
}

EnvironmentConfig env_from(const ChannelConfig& config) {
    EnvironmentConfig env;
    env.sound_speed_m_s  = config.sound_speed_m_s;
    env.saltwater        = config.saltwater;
    env.spreading_factor = config.spreading_factor;
    return env;
}

} // namespace

GeometricTapSource::GeometricTapSource(const ChannelConfig& config,
                                       float                source_center_fc_hz,
                                       uint32_t             sample_rate)
    : scene_(config.geometry, env_from(config))
    , cfg_initial_range_m_(config.initial_range_m)
    , range_m_(config.range_m)
    , velocity_m_s_(config.velocity_radial_m_s)
    , acceleration_m_s2_(config.acceleration_radial_m_s2)
    , r_min_m_(config.geometry.r_min_m)
    , r_max_m_(config.geometry.r_max_m)
    , center_fc_hz_(source_center_fc_hz)
    , saltwater_(config.saltwater)
    , sample_rate_(sample_rate)
{
    const double sound_speed = clamp_sound_speed(config.sound_speed_m_s);

    // Anchor base_delay at the shortest enabled path at r_min so every
    // tap's excess delta_samples stays >= 0 across the R envelope.
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths_rmin{};
    const std::size_t n_rmin = scene_.compute_paths(r_min_m_, paths_rmin);
    if (n_rmin == 0) {
        throw std::invalid_argument(
            "Channel: geometric mode requires at least one enabled path");
    }
    r_min_anchor_len_m_ = paths_rmin[0].length_m;
    base_delay_samples_ = static_cast<size_t>(
        std::round(static_cast<double>(r_min_anchor_len_m_) /
                   sound_speed * sample_rate_));

    // Worst-case tap excess: longest path at r_max minus the r_min anchor.
    // Drives PairBuffer sizing.
    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths_rmax{};
    const std::size_t n_rmax = scene_.compute_paths(r_max_m_, paths_rmax);
    if (n_rmax == 0) {
        throw std::invalid_argument(
            "Channel: geometric mode requires at least one enabled path "
            "at r_max");
    }
    const float worst_len = paths_rmax[n_rmax - 1].length_m;
    const float worst_excess_s = std::max(
        0.0f, (worst_len - r_min_anchor_len_m_) /
              static_cast<float>(sound_speed));
    max_tap_delta_samples_ = static_cast<size_t>(
        std::round(worst_excess_s * sample_rate_));

    // Path count is stable across blocks; report max(n_rmin, n_rmax) for
    // safety.
    tap_count_ = std::max(n_rmin, n_rmax);

    // Worst read-back behind the SourceDelayLine producer: the longest
    // enabled path's excess at the worst R in [r_min, r_max].
    const double rmin_worst_excess_s = std::max(0.0,
        (static_cast<double>(paths_rmin[n_rmin - 1].length_m) -
         static_cast<double>(r_min_anchor_len_m_)) / sound_speed);
    const double rmax_worst_excess_s = std::max(0.0,
        (static_cast<double>(worst_len) -
         static_cast<double>(r_min_anchor_len_m_)) / sound_speed);
    const size_t rmin_worst_samples = static_cast<size_t>(
        std::ceil(rmin_worst_excess_s * sample_rate_));
    const size_t rmax_worst_samples = static_cast<size_t>(
        std::ceil(rmax_worst_excess_s * sample_rate_));
    sdl_worst_excess_samples_ =
        std::max(rmin_worst_samples, rmax_worst_samples);

    // Sane taps before the first on_message_start().
    initial_range_m_ =
        (cfg_initial_range_m_ > 0.0f) ? cfg_initial_range_m_ : range_m_;
}

void GeometricTapSource::on_message_start() {
    initial_range_m_ =
        (cfg_initial_range_m_ > 0.0f) ? cfg_initial_range_m_ : range_m_;
}

float GeometricTapSource::range_at_source_time(double t_seconds) const {
    const double v = static_cast<double>(velocity_m_s_);
    const double a = static_cast<double>(acceleration_m_s2_);
    return static_cast<float>(
        static_cast<double>(initial_range_m_)
        + v * t_seconds + 0.5 * a * t_seconds * t_seconds);
}

size_t GeometricTapSource::taps_at(double t_seconds, SourcedTap* out,
                                   size_t capacity) {
    // Clamp to the configured envelope rather than throwing — the sized
    // PairBuffer / SourceDelayLine assume reads stay in that window.
    const float r_clamped = std::clamp(range_at_source_time(t_seconds),
                                       r_min_m_, r_max_m_);

    std::array<PathTap, MAX_GEOMETRIC_PATHS> paths{};
    const std::size_t n = scene_.compute_paths(r_clamped, paths);
    const std::size_t n_out = std::min(n, capacity);

    for (std::size_t i = 0; i < n_out; ++i) {
        const auto rp = scene_.resolve(paths[i], r_min_anchor_len_m_,
                                       center_fc_hz_, saltwater_,
                                       static_cast<float>(sample_rate_));
        out[i].delta_samples_frac = rp.delta_samples_frac;
        out[i].gain               = rp.gain_linear;
    }
    return n_out;
}

} // namespace openCREST
