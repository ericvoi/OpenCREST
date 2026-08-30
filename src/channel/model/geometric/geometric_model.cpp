#include "channel/model/geometric/geometric_model.hpp"

#include "channel/model/geometric/geometric_tap_source.hpp"
#include "channel/renderer/tap_source_renderer.hpp"

#include <memory>
#include <utility>

#include <spdlog/spdlog.h>

namespace openCREST {

std::unique_ptr<ChannelRenderer> make_geometric_renderer(
    const ChannelConfig& config, const ChannelBuildContext& ctx) {

    // The acoustic chain (spreading + Thorp) comes from GeometricTapSource
    // per-block. Only the AFE-electrical chain and the additive gain_db trim /
    // receive boost multiply on top.
    const double total_db = afe_chain_gain_db(config, ctx);

    auto source = std::make_unique<GeometricTapSource>(
        config, ctx.source_cal.center_freq_hz, ctx.sample_rate);

    // Base delay anchors at the shortest enabled path at r_min, not at a bulk
    // range/c delay, so every tap's excess stays >= 0 across the R envelope.
    const size_t base_delay_samples    = source->base_delay_samples();
    const size_t max_tap_delta_samples = source->max_tap_delta_samples();
    const float  r_min_anchor_len_m    = source->r_min_anchor_len_m();

    TapSourceRenderer::Params p;
    p.afe_chain_gain_linear    = afe_chain_gain_linear(config, ctx);
    p.base_delay_samples       = base_delay_samples;
    p.sample_rate              = ctx.sample_rate;
    p.clock_offset_ppm         = config.clock_offset_ppm;
    p.sdl_worst_excess_samples = source->sdl_worst_excess_samples();
    p.tap_source               = std::move(source);

    auto renderer = std::make_unique<TapSourceRenderer>(std::move(p));

    spdlog::info(
        "Channel {} → {}: mode=geometric, AFE_dB={:.2f}, "
        "fc={:.1f}kHz, base_delay={} (anchor at r_min={:.1f}m direct={:.2f}m), "
        "max_tap_delta={} (worst-path at r_max={:.1f}m), "
        "source_delay_line_capacity={}",
        config.from_modem, config.to_modem,
        total_db, ctx.source_cal.center_freq_hz / 1000.0,
        base_delay_samples, config.geometry.r_min_m,
        r_min_anchor_len_m,
        max_tap_delta_samples, config.geometry.r_max_m,
        renderer->source_delay_line_capacity());

    return renderer;
}

} // namespace openCREST
