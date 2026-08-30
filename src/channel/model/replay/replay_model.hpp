#pragma once

#include <memory>

#include "channel/model/channel_model.hpp"
#include "channel/renderer/channel_renderer.hpp"
#include "config/scenario.hpp"

namespace openCREST {

// Replay model: per-tap delay and amplitude trajectories measured offline and
// stored in an .octt file. Doppler emerges from the recorded time-varying
// delays, rendered at the modem's actual band. Spreading and absorption are
// baked into the measured gains, so only the AFE-electrical chain multiplies
// on top; the bulk propagation delay comes from range_m or the
// propagation_delay_s override, with recorded delays being excess over it.
//
// Builds a TapSourceRenderer around a ReplayTapSource. Reads the trajectory
// file at construction; throws TapTrajectoryError if it is missing or
// malformed.
std::unique_ptr<ChannelRenderer> make_replay_renderer(
    const ChannelConfig& config, const ChannelBuildContext& ctx);

} // namespace openCREST
