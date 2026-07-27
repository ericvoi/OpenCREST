#pragma once

#include <cstdint>
#include <string>

#include "channel/tap_source.hpp"
#include "core/tap_trajectory.hpp"
#include "config/scenario.hpp"

namespace openCREST {

// Measured tap source: replays per-tap delay/amplitude trajectories from
// an .octt file. Doppler emerges from the recorded time-varying delays,
// rendered at the modem's actual band by the per-tap Farrow pipeline.
//
// Replay clock: the record does not advance between messages. Each message
// maps intra-message time t onto record time message_offset + t, where the
// offset is fixed (config offset_s every message) or advancing (continues
// where the previous message — including its multipath-tail drain — ended,
// wrapping to 0 when the remaining record is shorter than the configured
// dead zone).
//
// Past the end of the record, each tap's amplitude ramps linearly from its
// final-frame value to zero across one frame interval and stays silent;
// delay holds its final value. The first past-end query of a message logs
// a warning: the acoustic output of the message is truncated.
class ReplayTapSource final : public TapSource {
public:
    ReplayTapSource(TapTrajectory       trajectory,   // moved in; owned
                    const ReplayConfig& config,
                    uint32_t            sample_rate,
                    std::string         channel_label);

    void   on_message_start() override;
    size_t taps_at(double t_seconds, SourcedTap* out,
                   size_t capacity) override;
    size_t tap_count_max()         const override {
        return trajectory_.tap_count();
    }
    size_t max_tap_delta_samples() const override {
        return max_tap_delta_samples_;
    }

    // Record time mapped to intra-message t = 0. Diagnostics/tests.
    double message_offset_s() const { return message_offset_s_; }

    // Trajectory header pass-throughs (diagnostics/logging).
    double dt_s()        const { return trajectory_.dt_s(); }
    double duration_s()  const { return trajectory_.duration_s(); }
    double fc_meas_hz()  const { return trajectory_.fc_meas_hz(); }

private:
    TapTrajectory trajectory_;
    double        offset_s_;
    bool          advance_per_message_;
    double        wrap_if_remaining_lt_s_;
    double        sample_rate_;
    std::string   channel_label_;
    size_t        max_tap_delta_samples_ = 0;

    double message_offset_s_ = 0.0;  // record time at intra-message t = 0
    double max_t_seen_s_     = 0.0;  // high-water of taps_at t this message
    bool   started_once_     = false;
    bool   past_end_warned_  = false;
};

} // namespace openCREST
