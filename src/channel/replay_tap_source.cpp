#include "channel/replay_tap_source.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <spdlog/spdlog.h>

namespace openCREST {

ReplayTapSource::ReplayTapSource(TapTrajectory       trajectory,
                                 const ReplayConfig& config,
                                 uint32_t            sample_rate,
                                 std::string         channel_label)
    : trajectory_(std::move(trajectory))
    , offset_s_(config.offset_s)
    , advance_per_message_(config.advance_per_message)
    , wrap_if_remaining_lt_s_(config.wrap_if_remaining_lt_s)
    , sample_rate_(static_cast<double>(sample_rate))
    , channel_label_(std::move(channel_label))
{
    max_tap_delta_samples_ = static_cast<size_t>(
        std::ceil(trajectory_.max_delay_s() * sample_rate_));
    message_offset_s_ = offset_s_;

    if (offset_s_ >= trajectory_.duration_s()) {
        spdlog::warn(
            "Channel {}: replay offset_s {:.3f} is at or past the end of "
            "the {:.3f}s record — messages will be silent",
            channel_label_, offset_s_, trajectory_.duration_s());
    }
}

void ReplayTapSource::on_message_start() {
    if (!advance_per_message_) {
        message_offset_s_ = offset_s_;
    } else {
        if (started_once_) {
            message_offset_s_ += max_t_seen_s_;
        } else {
            message_offset_s_ = offset_s_;
        }
        if (wrap_if_remaining_lt_s_ > 0.0
            && trajectory_.duration_s() - message_offset_s_
                   < wrap_if_remaining_lt_s_) {
            spdlog::info(
                "Channel {}: replay record remainder {:.3f}s is inside the "
                "{:.3f}s dead zone — wrapping to record start",
                channel_label_,
                trajectory_.duration_s() - message_offset_s_,
                wrap_if_remaining_lt_s_);
            message_offset_s_ = 0.0;
        }
    }
    started_once_    = true;
    max_t_seen_s_    = 0.0;
    past_end_warned_ = false;
}

size_t ReplayTapSource::taps_at(double t_seconds, SourcedTap* out,
                                size_t capacity) {
    max_t_seen_s_ = std::max(max_t_seen_s_, t_seconds);

    const double record_t = message_offset_s_ + t_seconds;
    const double duration = trajectory_.duration_s();

    // Past-end amplitude ramp: final-frame value scaled down to zero
    // across one frame interval. sample() clamps time, so querying at
    // record_t past the end yields the held final-frame state.
    double ramp = 1.0;
    if (record_t > duration) {
        if (!past_end_warned_) {
            past_end_warned_ = true;
            spdlog::warn(
                "Channel {}: message exceeds replay record; acoustic "
                "output truncated at {:.3f}s of record time",
                channel_label_, duration);
        }
        ramp = std::max(0.0, 1.0 - (record_t - duration) / trajectory_.dt_s());
    }

    const size_t n = std::min<size_t>(trajectory_.tap_count(), capacity);
    for (size_t k = 0; k < n; ++k) {
        const auto s = trajectory_.sample(k, record_t);
        out[k].delta_samples_frac = s.delay_s * sample_rate_;
        out[k].gain               = s.amplitude * static_cast<float>(ramp);
    }
    return n;
}

} // namespace openCREST
