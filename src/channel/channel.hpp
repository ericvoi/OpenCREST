#pragma once
#include <vector>
#include <memory>
#include <cstddef>
#include <cstdint>
#include "config/scenario.hpp"
#include "core/types.hpp"
#include "dsp/farrow_resampler.hpp"
#include "dsp/hilbert_filter.hpp"
#include "dsp/per_tap_farrow.hpp"
#include "dsp/source_delay_line.hpp"
#include "channel/pair_buffer.hpp"
#include "channel/tap_source.hpp"

namespace openCREST {

// Single directed acoustic path: one source modem -> one receiver modem.
//
// Push-style: SourceWorker hands in source ADC samples (already converted to
// float) and Channel scatters multipath tap contributions into the
// PairBuffer (which is the propagation-delay buffer).
//
// Per process() call:
//   1. FarrowResampler applies Doppler compression/expansion.
//   2. For each tap k: scatter (farrow_out * tap_gain_k) into pair_buffer
//      at offset (source_pos + tap_delta_k).
//   3. commit_source_progress.
//
// Doppler ratio is recomputed at every process() call from
//   ratio = (1 + delta_clock) * (1 + v(t)/c),   v(t) = v0 + a*t
// where t is intra-message source-time. Per-block update is fine: 1 m/s^2
// over a 256-sample block shifts the ratio by ~3e-7.
class Channel {
public:
    Channel(const ChannelConfig&   config,
            const CalibrationData& source_cal,
            const CalibrationData& receiver_cal,
            const TransducerSpec&  source_transducer,
            const TransducerSpec&  receiver_transducer,
            float                  receive_boost_db = 0.0f);

    ~Channel();

    // Source modem entered TX. Resets Farrow state, recomputes the Doppler
    // ratio, and forwards to pair_buffer.begin_message().
    // `absolute_first_origin = true` selects the clock-tracker arrival-
    // alignment policy (caller has folded propagation delay into the gap);
    // default false applies base_delay automatically on the first message.
    void on_message_start(PairBuffer& pair_buffer,
                          size_t      inter_message_gap_samples = 0,
                          bool        absolute_first_origin     = false);

    // Source modem left TX. Drains Farrow history with zero input and
    // advances the commit watermark by max_tap_delta_samples to expose the
    // multipath tail.
    void on_message_end(PairBuffer& pair_buffer);

    // Process up to `count` source samples ([-1,+1] float). Returns the
    // number of samples Farrow consumed; caller retains the unconsumed
    // tail. Allocation-free.
    size_t process(const float* samples, size_t count, PairBuffer& pair_buffer);

    // Conservative upper bound on input samples the caller should batch per
    // process() call.
    size_t input_needed_for_batch() const;

    const ChannelConfig& config() const { return config_; }

    // Maximum tap delay in samples (relative to the zero-delay reference).
    size_t max_tap_delta_samples() const { return max_tap_delta_samples_; }

    // Base propagation delay (range_m * sample_rate / sound_speed).
    size_t base_delay_samples()    const { return base_delay_samples_; }

    // Same delay in seconds (independent of receiver Fs). Used by the
    // arrival-alignment math in clock-tracker mode.
    double propagation_delay_seconds() const {
        return (sample_rate_ > 0)
            ? (static_cast<double>(base_delay_samples_) /
               static_cast<double>(sample_rate_))
            : 0.0;
    }

    // Instantaneous Doppler ratio at the current intra-message time.
    // Diagnostics only.
    double current_doppler_ratio() const;

private:
    struct ResolvedTap {
        // Delay in source-rate samples. Static mode stores integer
        // (delay_s * sample_rate); geometric mode stores fractional
        // excess over the r_min direct-path anchor.
        double delta_samples_frac;
        float  gain;       // path-loss * tap_gain * cos(phase_rad)
        float  gain_imag;  // path-loss * tap_gain * sin(phase_rad);
                           // any non-zero value enables the Hilbert path
                           // for the whole channel.
    };

    // Static-mode scatter: bulk Farrow output -> PairBuffer at integer
    // (source_pos + tap_delta). Also handles the complex/Hilbert path.
    void scatter_taps(size_t source_position_start,
                      const float* farrow_out, size_t n,
                      PairBuffer& pair_buffer);

    // Read-style branch of process() (geometric / replay): writes source
    // samples 1:1 into source_delay_line_ and produces receiver-rate output
    // via per-tap PerTapFarrow reads, with tap states supplied by
    // tap_source_ at block start/end. Returns input consumed (== count).
    size_t process_tap_source(const float* samples, size_t count,
                              PairBuffer& pair_buffer);

    // Drain the read-style multipath tail with zero input.
    void drain_tap_source_tail(PairBuffer& pair_buffer);

    // Size the read-style scratch buffers and the SourceDelayLine
    // (worst-case excess + Catmull-Rom, clock-drift, and block margins).
    void init_read_style_buffers(size_t tap_count,
                                 size_t sdl_worst_excess_samples);

    ChannelConfig            config_;
    dsp::FarrowResampler     resampler_;
    dsp::HilbertFilter       hilbert_;     // shared by all complex taps
    std::vector<ResolvedTap> taps_;
    size_t                   max_tap_delta_samples_ = 0;
    size_t                   base_delay_samples_    = 0;
    uint32_t                 sample_rate_           = 500'000;

    // True iff any tap has non-zero imaginary gain. Real-only channels
    // skip the Hilbert pair and its group-delay offset.
    bool                     needs_hilbert_         = false;

    // Per-message state (reset by on_message_start).
    size_t source_samples_processed_ = 0;

    // Read-style tap provider (geometric / replay modes); null in static
    // mode. Tap delays/gains are re-queried each block at the block's
    // start/end times. The Farrow ratio in these modes carries only the
    // crystal-clock offset — the (1 + v/c) shift emerges naturally from
    // time-varying tap deltas.
    std::unique_ptr<TapSource> tap_source_;
    float  afe_chain_gain_       = 1.0f;             // AFE-only linear scalar

    // Pre-allocated scratch.
    std::vector<float> resample_buf_;       // Farrow output, one batch
    std::vector<float> gained_buf_;         // farrow_out * tap.gain
    std::vector<float> hilbert_buf_;        // sized only when needs_hilbert_
    std::vector<float> delayed_real_buf_;   // sized only when needs_hilbert_

    // Read-style state (tap_source_ modes). Source samples land 1:1 in
    // source_delay_line_; each tap's PerTapFarrow reads at a fractional
    // position evolving across the block, so per-tap Doppler emerges
    // without invoking the bulk FarrowResampler.
    dsp::SourceDelayLine        source_delay_line_;
    std::vector<SourcedTap>     taps_start_buf_;
    std::vector<SourcedTap>     taps_end_buf_;
    std::vector<dsp::TapState>  tap_states_;
    std::vector<float>          per_tap_scratch_;
    // Receiver-rate output cursor for the current message.
    uint64_t                    pair_buffer_out_cursor_   = 0;
    // Source-rate position passed to PerTapFarrow. Double so non-zero
    // clock_offset_ppm drifts smoothly.
    double                      source_delay_line_cursor_ = 0.0;
};

} // namespace openCREST
