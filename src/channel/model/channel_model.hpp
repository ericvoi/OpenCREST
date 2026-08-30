#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "channel/renderer/channel_renderer.hpp"
#include "config/scenario.hpp"
#include "core/types.hpp"

namespace openCREST {

// Everything a channel model needs to turn its configuration into a renderer,
// beyond the ChannelConfig itself. Supplied by Channel; models never reach
// back into ScenarioConfig or the engine.
struct ChannelBuildContext {
    CalibrationData source_cal;
    CalibrationData receiver_cal;
    TransducerSpec  source_transducer;
    TransducerSpec  receiver_transducer;

    // Per-receiver auto-boost (dB) applied when the natural Wenz PSD at the
    // receiver's fc sits below the AFE noise floor + min_margin. Every channel
    // feeding that receiver is scaled by the same dB so SNR is preserved.
    float           receive_boost_db = 0.0f;

    // Source ADC rate; all delays and tap deltas are expressed in it.
    uint32_t        sample_rate      = 500'000;
};

// One propagation model, as seen by the rest of the system.
//
// A model owns everything specific to itself: how its configuration becomes a
// renderer, which gain chain applies, where its base delay comes from, and
// what it logs at startup. Nothing outside src/channel/model/<name>/ branches
// on which model is in use — ChannelEngine sizes PairBuffers from the
// renderer's reported extent, and SourceWorker drives the renderer through the
// ChannelRenderer interface.
//
// Adding a model means adding one folder plus one line in the registry table
// in channel_model.cpp.
struct ChannelModel {
    // Matches the scenario YAML `mode:` value. Diagnostics and logging.
    const char* name = "";

    // Build the renderer for one directed channel. Throws on invalid
    // configuration (e.g. a geometric scene with no enabled path, or an
    // unreadable replay trajectory).
    std::unique_ptr<ChannelRenderer> (*make_renderer)(
        const ChannelConfig&, const ChannelBuildContext&) = nullptr;
};

// Registry lookup. Throws std::invalid_argument for an unregistered mode.
const ChannelModel& find_channel_model(ChannelMode mode);

// ---------------------------------------------------------------------------
// Shared helpers for model implementations
// ---------------------------------------------------------------------------

// Sound speed with the "unset or nonsensical" fallback every model applies.
double clamp_sound_speed(float configured);

// Bulk propagation delay in source-rate samples: propagation_delay_s when
// given (>= 0), otherwise range_m / sound_speed. Used by every model whose
// base delay is a scalar bulk delay with tap deltas measured as excess over
// it — that is, all of them except geometric, which anchors on the shortest
// enabled ray at r_min instead.
size_t bulk_propagation_delay_samples(const ChannelConfig& config,
                                      uint32_t             sample_rate);

// AFE-electrical chain scalar (linear) for models whose tap gains already
// carry spreading and absorption: the electronics chain, plus the additive
// gain_db trim and the per-receiver receive boost.
float afe_chain_gain_linear(const ChannelConfig&       config,
                            const ChannelBuildContext& ctx);

// The same chain in dB, for the startup log line.
double afe_chain_gain_db(const ChannelConfig&       config,
                         const ChannelBuildContext& ctx);

} // namespace openCREST
