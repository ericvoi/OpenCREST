#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "channel/model/tap_source.hpp"
#include "channel/pair_buffer.hpp"
#include "channel/renderer/channel_renderer.hpp"
#include "dsp/per_tap_farrow.hpp"
#include "dsp/source_delay_line.hpp"

namespace openCREST {

// Read-style renderer driven by a TapSource.
//
// Source samples are written 1:1 into a SourceDelayLine; receiver-rate output
// is produced by per-tap PerTapFarrow reads at fractional positions that
// evolve across each block. The tap set is re-queried from the TapSource at
// the block's start and end times and linearly interpolated between them, so
// range-rate Doppler emerges from time-varying tap delays rather than a bulk
// resampling ratio. The only bulk ratio applied here is the crystal-clock
// offset, folded into each tap's end-delay.
//
// Shared by every tap-based channel model (geometric, replay, and any future
// live/digital-twin source): the model supplies the TapSource and the resolved
// AFE gain, and this class owns all of the rendering.
class TapSourceRenderer final : public ChannelRenderer {
public:
    struct Params {
        // Tap provider. Must be non-null; the renderer takes ownership.
        std::unique_ptr<TapSource> tap_source;

        // AFE-electrical chain scalar (gain_db trim and receive boost folded
        // in). TapSource gains are model-relative acoustic gains; this
        // multiplies on top.
        float    afe_chain_gain_linear = 1.0f;

        size_t   base_delay_samples    = 0;
        uint32_t sample_rate           = 500'000;
        float    clock_offset_ppm      = 0.0f;

        // Worst read-back distance behind the SourceDelayLine producer, in
        // source-rate samples. Models that can bound this more tightly than
        // max_tap_delta_samples() should do so; the renderer adds its own
        // interpolation, clock-drift, and block margins on top.
        size_t   sdl_worst_excess_samples = 0;
    };

    explicit TapSourceRenderer(Params params);
    ~TapSourceRenderer() override;

    void   on_message_start() override;
    size_t process(const float* samples, size_t count,
                   PairBuffer& pair_buffer) override;
    void   on_message_end(PairBuffer& pair_buffer) override;
    size_t input_needed_for_batch() const override;

    size_t base_delay_samples()    const override { return base_delay_samples_; }
    size_t max_tap_delta_samples() const override {
        return max_tap_delta_samples_;
    }

    // Range-rate Doppler lives in the tap deltas, so the bulk ratio carries
    // only the crystal-clock offset.
    double current_doppler_ratio() const override {
        return 1.0 + static_cast<double>(clock_offset_ppm_) * 1e-6;
    }

    // SourceDelayLine capacity after margin sizing. Model logging / tests.
    size_t source_delay_line_capacity() const {
        return source_delay_line_.capacity();
    }

private:
    // Feed zeros so every tap's read pointer catches up with the producer and
    // the multipath tail fully decays.
    void drain_tail(PairBuffer& pair_buffer);

    std::unique_ptr<TapSource> tap_source_;

    float    afe_chain_gain_       = 1.0f;
    size_t   base_delay_samples_   = 0;
    size_t   max_tap_delta_samples_ = 0;
    uint32_t sample_rate_          = 500'000;
    float    clock_offset_ppm_     = 0.0f;

    dsp::SourceDelayLine       source_delay_line_;
    std::vector<SourcedTap>    taps_start_buf_;
    std::vector<SourcedTap>    taps_end_buf_;
    std::vector<dsp::TapState> tap_states_;
    std::vector<float>         per_tap_scratch_;

    // Receiver-rate output cursor for the current message.
    uint64_t pair_buffer_out_cursor_ = 0;
    // Source-rate position passed to PerTapFarrow. Double so a non-zero
    // clock_offset_ppm drifts smoothly.
    double   source_delay_line_cursor_ = 0.0;
};

} // namespace openCREST
