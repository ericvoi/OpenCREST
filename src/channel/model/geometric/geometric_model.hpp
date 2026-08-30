#pragma once

#include <memory>

#include "channel/model/channel_model.hpp"
#include "channel/renderer/channel_renderer.hpp"
#include "config/scenario.hpp"

namespace openCREST {

// Geometric model: a method-of-images scene evaluated at R(t), yielding taps
// whose delays shrink or grow with range so closing-range Doppler emerges
// naturally. Spreading and Thorp absorption come from the scene per path, so
// only the AFE-electrical chain multiplies on top.
//
// Builds a TapSourceRenderer around a GeometricTapSource. Throws
// std::invalid_argument when the scene has no enabled path at r_min or r_max.
std::unique_ptr<ChannelRenderer> make_geometric_renderer(
    const ChannelConfig& config, const ChannelBuildContext& ctx);

} // namespace openCREST
