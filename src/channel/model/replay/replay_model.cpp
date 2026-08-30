#include "channel/model/replay/replay_model.hpp"

#include "channel/model/replay/replay_tap_source.hpp"
#include "channel/model/replay/tap_trajectory.hpp"
#include "channel/renderer/tap_source_renderer.hpp"

#include <memory>
#include <utility>

#include <spdlog/spdlog.h>

namespace openCREST {

std::unique_ptr<ChannelRenderer> make_replay_renderer(
    const ChannelConfig& config, const ChannelBuildContext& ctx) {

    // The acoustic path structure (delays, relative gains) comes from the
    // recorded trajectory. Only the AFE-electrical chain and the additive
    // gain_db trim / receive boost multiply on top — spreading and absorption
    // are baked into the measured gains.
    const double total_db = afe_chain_gain_db(config, ctx);

    // Init-time file I/O; a malformed or missing file fails scenario startup
    // with TapTrajectoryError.
    auto source = std::make_unique<ReplayTapSource>(
        TapTrajectory::load(config.replay.trajectory_path),
        config.replay, ctx.sample_rate,
        config.from_modem + " → " + config.to_modem);

    const size_t max_tap_delta_samples = source->max_tap_delta_samples();
    const size_t tap_count             = source->tap_count_max();
    const double dt_s                  = source->dt_s();
    const double duration_s            = source->duration_s();
    const double fc_meas_hz            = source->fc_meas_hz();

    // Base propagation delay as in static mode: propagation_delay_s overrides
    // the range-derived delay (trajectory delays are excess over this).
    const size_t base_delay_samples =
        bulk_propagation_delay_samples(config, ctx.sample_rate);

    TapSourceRenderer::Params p;
    p.afe_chain_gain_linear    = afe_chain_gain_linear(config, ctx);
    p.base_delay_samples       = base_delay_samples;
    p.sample_rate              = ctx.sample_rate;
    p.clock_offset_ppm         = config.clock_offset_ppm;
    p.sdl_worst_excess_samples = max_tap_delta_samples;
    p.tap_source               = std::move(source);

    auto renderer = std::make_unique<TapSourceRenderer>(std::move(p));

    spdlog::info(
        "Channel {} → {}: mode=replay, AFE_dB={:.2f}, file='{}' "
        "(taps={}, dt={:.1f}ms, duration={:.2f}s, fc_meas={:.1f}kHz), "
        "base_delay={}, max_tap_delta={}, source_delay_line_capacity={}",
        config.from_modem, config.to_modem,
        total_db, config.replay.trajectory_path,
        tap_count, dt_s * 1000.0, duration_s, fc_meas_hz / 1000.0,
        base_delay_samples, max_tap_delta_samples,
        renderer->source_delay_line_capacity());

    return renderer;
}

} // namespace openCREST
