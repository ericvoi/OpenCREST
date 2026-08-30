#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>

#include "config/scenario.hpp"
#include "core/types.hpp"
#include "channel/pair_buffer.hpp"
#include "channel/renderer/channel_renderer.hpp"

namespace openCREST {

// Single directed acoustic path: one source modem -> one receiver modem.
//
// Push-style: SourceWorker hands in source ADC samples (already converted to
// float) and Channel renders receiver-time contributions into the PairBuffer
// (which is the propagation-delay buffer).
//
// Channel itself is model-agnostic. The constructor asks the model registry
// (channel/model/channel_model.hpp) for the model named by config.mode and
// hands it the calibration context; the model returns a ChannelRenderer that
// owns every model-specific decision — gain chain, base delay, tap supply, and
// signal path. Channel adds only the PairBuffer message bookkeeping that is
// common to all models.
//
// Everything model-specific lives under src/channel/model/<name>/. To add a
// propagation model, add a folder there and one registry entry; no file
// outside it needs to change.
class Channel {
public:
    // Throws if the model rejects the configuration (e.g. a geometric scene
    // with no enabled path, or an unreadable replay trajectory file).
    Channel(const ChannelConfig&   config,
            const CalibrationData& source_cal,
            const CalibrationData& receiver_cal,
            const TransducerSpec&  source_transducer,
            const TransducerSpec&  receiver_transducer,
            float                  receive_boost_db = 0.0f);

    ~Channel();

    // Source modem entered TX. Resets the renderer's per-message state and
    // forwards to pair_buffer.begin_message().
    // `absolute_first_origin = true` selects the clock-tracker arrival-
    // alignment policy (caller has folded propagation delay into the gap);
    // default false applies base_delay automatically on the first message.
    void on_message_start(PairBuffer& pair_buffer,
                          size_t      inter_message_gap_samples = 0,
                          bool        absolute_first_origin     = false);

    // Source modem left TX. Drains the renderer's filter state and exposes
    // the multipath tail.
    void on_message_end(PairBuffer& pair_buffer);

    // Process up to `count` source samples ([-1,+1] float). Returns the number
    // of samples consumed; caller retains the unconsumed tail.
    // Allocation-free.
    size_t process(const float* samples, size_t count, PairBuffer& pair_buffer);

    // Conservative upper bound on input samples the caller should batch per
    // process() call.
    size_t input_needed_for_batch() const;

    const ChannelConfig& config() const { return config_; }

    // Maximum tap delay in samples (relative to the base delay).
    size_t max_tap_delta_samples() const;

    // Base propagation delay in source-rate samples.
    size_t base_delay_samples() const;

    // Total write extent of one message beyond its own length, in receiver-
    // rate samples: base delay plus the longest tap. ChannelEngine sizes
    // PairBuffers from the worst of these across all channels — no model
    // knowledge required.
    size_t write_extent_samples() const {
        return base_delay_samples() + max_tap_delta_samples();
    }

    // Base delay in seconds (independent of receiver Fs). Used by the
    // arrival-alignment math in clock-tracker mode.
    double propagation_delay_seconds() const {
        return (sample_rate_ > 0)
            ? (static_cast<double>(base_delay_samples()) /
               static_cast<double>(sample_rate_))
            : 0.0;
    }

    // Instantaneous Doppler ratio at the current intra-message time.
    // Diagnostics only.
    double current_doppler_ratio() const;

private:
    ChannelConfig                    config_;
    uint32_t                         sample_rate_ = 500'000;
    std::unique_ptr<ChannelRenderer> renderer_;
};

} // namespace openCREST
