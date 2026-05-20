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

namespace openCREST {

class GeometricScene;

// Single directed acoustic path: one source modem → one receiver modem.
//
// In Phase 2, Channel is *push-style*: the SourceWorker hands it source ADC
// samples (already calibrated to float) and Channel scatters its multipath
// tap contributions directly into the PairBuffer. The propagation-delay
// buffer is the PairBuffer.
//
// Pipeline per process() call:
//   1. FarrowResampler — Doppler compression/expansion of the source samples
//   2. For each tap k:
//        gained = farrow_out * tap_gain_k
//        pair_buffer.scatter_add(source_pos + tap_delta_k, gained, n)
//   3. pair_buffer.commit_source_progress(source_pos + n)
//
// Per-pair Farrow ratio
// ---------------------
// The Doppler ratio is recomputed at the start of every process() call from
//
//   ratio = (1 + δ_clock) × (1 + v(t)/c)
//   v(t)  = v0 + a·t       (t = source seconds elapsed since on_message_start)
//
// so constant radial acceleration smoothly modulates the resampling ratio
// over the duration of one message. Per-block update is sufficient for any
// physically realistic acceleration (1 m/s² over a 256-sample / 0.5 ms
// block changes the ratio by ~3·10⁻⁷).
class Channel {
public:
    Channel(const ChannelConfig&   config,
            const CalibrationData& source_cal,
            const CalibrationData& receiver_cal,
            const TransducerSpec&  source_transducer,
            const TransducerSpec&  receiver_transducer,
            float                  receive_boost_db = 0.0f);

    // Out-of-line so std::unique_ptr<GeometricScene> can use the forward
    // declaration above.
    ~Channel();

    // ----------------------------------------------------------------------
    // Message lifecycle (called by SourceWorker on the source TX edges)
    // ----------------------------------------------------------------------

    // Source modem entered TX. Resets Farrow state, recomputes the Doppler
    // ratio from the channel config, and calls
    //   pair_buffer.begin_message(inter_message_gap_samples,
    //                              absolute_first_origin)
    // The gap parameter expresses source-time idle since the previous
    // TX-exit on the same source. For Phase 2 MVP, callers pass 0.
    //
    // `absolute_first_origin = true` is the clock-tracker arrival-alignment
    // path: caller has folded propagation delay (and pipeline-depth
    // corrections) into the gap, and the PairBuffer must NOT auto-apply
    // base_delay on the first message. Default false preserves the legacy
    // PID-mode behavior.
    void on_message_start(PairBuffer& pair_buffer,
                          size_t      inter_message_gap_samples = 0,
                          bool        absolute_first_origin     = false);

    // Source modem left TX. Drains any in-flight Farrow history (feeds zero
    // input through the resampler so trailing real-input samples are
    // interpolated and scattered through the taps), then advances the
    // PairBuffer commit watermark by max_tap_delta_samples so the multipath
    // tail becomes visible to the receiver.
    void on_message_end(PairBuffer& pair_buffer);

    // ----------------------------------------------------------------------
    // Per-batch processing
    // ----------------------------------------------------------------------

    // Process up to `count` source samples (float, [-1,+1]). Returns the
    // number of source samples consumed by Farrow (the SourceWorker must
    // retain unconsumed input for the next call).
    //
    // Internally:
    //   * Runs Farrow output-driven up to one resample-buffer batch
    //   * For each Farrow output range [s, s+m) within this call,
    //     scatters every tap into pair_buffer
    //   * Calls pair_buffer.commit_source_progress(s+m) at the end
    //
    // Allocation-free; uses pre-allocated scratch.
    size_t process(const float* samples, size_t count, PairBuffer& pair_buffer);

    // Conservative upper bound on the number of source samples the
    // SourceWorker should batch before calling process(). Accounts for the
    // Farrow ratio and the resample-buffer scratch size.
    size_t input_needed_for_batch() const;

    const ChannelConfig& config() const { return config_; }

    // Maximum tap delay in samples (relative to the zero-delay reference).
    // Used by ChannelEngine for sizing the destination PairBuffer.
    size_t max_tap_delta_samples() const { return max_tap_delta_samples_; }

    // Base propagation delay in samples (range_m × sample_rate / sound_speed).
    // Used by ChannelEngine for sizing the destination PairBuffer.
    size_t base_delay_samples()    const { return base_delay_samples_; }

    // Same delay expressed in seconds (independent of receiver Fs).
    // Used by SourceWorker's arrival-alignment math in clock-tracker
    // mode: the target message-start arrival time on the receiver is
    // T_tx_start_source + propagation_delay_seconds.
    double propagation_delay_seconds() const {
        return (sample_rate_ > 0)
            ? (static_cast<double>(base_delay_samples_) /
               static_cast<double>(sample_rate_))
            : 0.0;
    }

    // Instantaneous Doppler ratio for the current intra-message time
    // (source_samples_processed_ / sample_rate). Public for diagnostics
    // and tests; the engine never needs to call it.
    double current_doppler_ratio() const;

private:
    struct ResolvedTap {
        // Delay in source-rate samples relative to the source-position
        // anchor. Static mode populates this with an integer (.delay_s ·
        // sample_rate, rounded); geometric mode populates with the raw
        // fractional excess over the r_min direct-path anchor.
        double delta_samples_frac;
        float  gain;       // path-loss × tap_gain × cos(phase_rad)
        float  gain_imag;  // path-loss × tap_gain × sin(phase_rad);
                           // applied via Hilbert pair when non-zero (any
                           // tap with non-zero phase enables the complex
                           // scatter path for the whole channel).
    };

    // Static-mode-only: bulk Farrow output scattered into the PairBuffer
    // at integer (source_pos + tap_delta) offsets. The complex/Hilbert
    // tap path also lives here. Geometric mode bypasses this entirely
    // and uses the SourceDelayLine + PerTapFarrow read-style path
    // (channel.cpp § geometric branch).
    void scatter_taps(size_t source_position_start,
                      const float* farrow_out, size_t n,
                      PairBuffer& pair_buffer);

    // Refresh `dst` from the geometric scene at horizontal range R, using
    // the existing tap count. Static mode never calls this.
    void recompute_geometric_taps_into(float range_m,
                                        std::vector<ResolvedTap>& dst) const;

    // Convenience wrapper: refresh taps_ in place.
    void recompute_geometric_taps(float range_m);

    // Geometric branch of process(): pushes source samples into
    // source_delay_line_ and produces receiver-rate output via per-tap
    // PerTapFarrow reads. Returns input samples consumed (== count).
    size_t process_geometric(const float* samples, size_t count,
                              PairBuffer& pair_buffer);

    // Drains the geometric tail by feeding zeros into source_delay_line_
    // and producing max_tap_delta_samples_ extra receiver-rate samples.
    void drain_geometric_tail(PairBuffer& pair_buffer);

    // R(t) = R_0 + v·t + 0.5·a·t² (only meaningful in geometric mode).
    float range_at_source_time(double t_seconds) const;

    ChannelConfig            config_;
    dsp::FarrowResampler     resampler_;
    dsp::HilbertFilter       hilbert_;     // shared by all complex taps
    std::vector<ResolvedTap> taps_;
    size_t                   max_tap_delta_samples_ = 0;
    size_t                   base_delay_samples_    = 0;
    uint32_t                 sample_rate_           = 500'000;

    // True iff any resolved tap has a non-zero imaginary gain. Enables the
    // Hilbert-pair scatter path. Real-only channels use the cheaper direct
    // scatter and incur no Hilbert group-delay offset.
    bool                     needs_hilbert_         = false;

    // Per-message state (reset by on_message_start)
    size_t source_samples_processed_ = 0;

    // ----- Geometric-mode state (null/zero in static mode) -----------------
    // Range evolves as R(t) = initial_range_m_ + v·t + 0.5·a·t² with
    // (v, a) sourced from config_.{velocity_radial_m_s,
    // acceleration_radial_m_s2}. Tap delays/gains are recomputed each
    // Farrow batch from R(t); the bulk Farrow Doppler ratio drops the
    // (1 + v/c) factor because that frequency shift now emerges naturally
    // from time-varying tap deltas.
    std::unique_ptr<GeometricScene> scene_;          // null in static mode
    float  initial_range_m_      = 0.0f;             // R_0 snapshot at message start
    float  source_center_fc_hz_  = 0.0f;             // cached from source_cal
    bool   saltwater_            = true;
    float  afe_chain_gain_       = 1.0f;             // AFE-only linear scalar
    float  r_min_anchor_len_m_   = 0.0f;             // shortest enabled path at r_min
    bool   geometric_            = false;

    // Pre-allocated scratch — never reallocated.
    std::vector<float> resample_buf_;       // Farrow output, one batch
    std::vector<float> gained_buf_;         // farrow_out × tap.gain, one tap
    std::vector<float> hilbert_buf_;        // Hilbert output (only sized
                                            // when needs_hilbert_)
    std::vector<float> delayed_real_buf_;   // input delayed by Hilbert group
                                            // delay (only sized when
                                            // needs_hilbert_)

    // ----- Geometric-mode read-style state (Session C) -----
    // Source samples land 1:1 in source_delay_line_; each tap's
    // PerTapFarrow reads at a fractional position evolving across the
    // block per dτ_k/dt — per-tap Doppler emerges naturally without
    // invoking the bulk FarrowResampler.
    dsp::SourceDelayLine        source_delay_line_;
    std::vector<ResolvedTap>    geom_taps_start_;   // taps at block-start R
    std::vector<ResolvedTap>    geom_taps_end_;     // taps at block-end R
    std::vector<dsp::TapState>  tap_states_;        // one per tap per block
    std::vector<float>          per_tap_scratch_;   // PROCESSING_BLOCK_SIZE
    // Receiver-rate output cursor for the current message (advances per
    // produced block; used as scatter offset and commit watermark).
    uint64_t                    pair_buffer_out_cursor_   = 0;
    // Source-rate position passed as out_pos_start to PerTapFarrow.
    // Tracked as double so non-zero clock_offset_ppm drifts smoothly.
    double                      source_delay_line_cursor_ = 0.0;
};

} // namespace openCREST
