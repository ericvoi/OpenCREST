#include "channel/model/channel_model.hpp"

#include "channel/model/geometric/geometric_model.hpp"
#include "channel/model/replay/replay_model.hpp"
#include "channel/model/static/static_model.hpp"
#include "dsp/physical_gain.hpp"

#include <cmath>
#include <stdexcept>

namespace openCREST {

namespace {

// The registry. This is the only place that names every model, and it holds
// no model logic — just the table.
const ChannelModel kStaticModel{"static", &make_static_renderer};
const ChannelModel kGeometricModel{"geometric", &make_geometric_renderer};
const ChannelModel kReplayModel{"replay", &make_replay_renderer};

} // namespace

const ChannelModel& find_channel_model(ChannelMode mode) {
    switch (mode) {
        case ChannelMode::Static:    return kStaticModel;
        case ChannelMode::Geometric: return kGeometricModel;
        case ChannelMode::Replay:    return kReplayModel;
    }
    throw std::invalid_argument("find_channel_model: unregistered ChannelMode");
}

double clamp_sound_speed(float configured) {
    return (configured > 0.0f) ? static_cast<double>(configured) : 1500.0;
}

size_t bulk_propagation_delay_samples(const ChannelConfig& config,
                                      uint32_t             sample_rate) {
    const double sound_speed = clamp_sound_speed(config.sound_speed_m_s);
    const double base_delay_seconds =
        (config.propagation_delay_s >= 0.0f)
            ? static_cast<double>(config.propagation_delay_s)
            : static_cast<double>(config.range_m) / sound_speed;
    return static_cast<size_t>(
        std::round(base_delay_seconds * sample_rate));
}

double afe_chain_gain_db(const ChannelConfig&       config,
                         const ChannelBuildContext& ctx) {
    const double chain_db = dsp::compute_channel_afe_chain_gain_db(
        ctx.source_cal, ctx.receiver_cal,
        ctx.source_transducer, ctx.receiver_transducer,
        config.rx_atten_idx);
    return chain_db
         + static_cast<double>(config.gain_db)
         + static_cast<double>(ctx.receive_boost_db);
}

float afe_chain_gain_linear(const ChannelConfig&       config,
                            const ChannelBuildContext& ctx) {
    return static_cast<float>(
        std::pow(10.0, afe_chain_gain_db(config, ctx) / 20.0));
}

} // namespace openCREST
