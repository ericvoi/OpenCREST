#pragma once

#include <memory>

#include "channel/model/channel_model.hpp"
#include "channel/renderer/channel_renderer.hpp"
#include "config/scenario.hpp"

namespace openCREST {

// Static model: the scenario supplies a flat multipath tap list directly.
// Path loss is computed once at the source's centre frequency and folded into
// every tap; Doppler comes from the bulk resampling ratio (crystal offset and
// constant radial velocity/acceleration).
//
// Unlike the geometric and replay models this one does not go through a
// TapSource — a fixed tap list is better served by bulk resampling plus
// integer scatter, so it builds a StaticRenderer instead.
std::unique_ptr<ChannelRenderer> make_static_renderer(
    const ChannelConfig& config, const ChannelBuildContext& ctx);

} // namespace openCREST
