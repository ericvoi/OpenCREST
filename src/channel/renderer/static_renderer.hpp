#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "channel/pair_buffer.hpp"
#include "channel/renderer/channel_renderer.hpp"
#include "core/types.hpp"
#include "dsp/farrow_resampler.hpp"
#include "dsp/hilbert_filter.hpp"

namespace openCREST {

// Write-style renderer for a fixed tap list.
//
// Per process() call:
//   1. FarrowResampler applies Doppler compression/expansion.
//   2. For each tap k: scatter (farrow_out * tap_gain_k) into the PairBuffer
//      at offset (source_pos + tap_delta_k).
//   3. commit_source_progress.
//
// Doppler ratio is recomputed at every process() call from
//   ratio = (1 + delta_clock) * (1 + v(t)/c),   v(t) = v0 + a*t
// where t is intra-message source-time. Per-block update is fine: 1 m/s^2
// over a 256-sample block shifts the ratio by ~3e-7.
class StaticRenderer final : public ChannelRenderer {
public:
    // Everything the renderer needs, already resolved to numbers by the
    // static channel model. No ScenarioConfig / ChannelConfig dependency.
    struct Params {
        // Taps as authored (delay_s / gain_linear / phase_rad). Gains are
        // relative; `channel_gain_linear` scales all of them.
        std::vector<MultipathTap> taps;

        // Total source-ADC-sample to receiver-DAC-sample linear scalar
        // (physical gain chain + gain_db trim + receive boost).
        float    channel_gain_linear = 1.0f;

        size_t   base_delay_samples  = 0;
        uint32_t sample_rate         = 500'000;

        // Doppler inputs. The bulk resampling ratio is
        // (1 + clock_offset_ppm*1e-6) * (1 + v(t)/sound_speed).
        float    clock_offset_ppm         = 0.0f;
        float    sound_speed_m_s          = 1500.0f;
        float    velocity_radial_m_s      = 0.0f;
        float    acceleration_radial_m_s2 = 0.0f;
    };

    explicit StaticRenderer(Params params);

    void   on_message_start() override;
    size_t process(const float* samples, size_t count,
                   PairBuffer& pair_buffer) override;
    void   on_message_end(PairBuffer& pair_buffer) override;
    size_t input_needed_for_batch() const override;

    size_t base_delay_samples()    const override { return base_delay_samples_; }
    size_t max_tap_delta_samples() const override {
        return max_tap_delta_samples_;
    }
    double current_doppler_ratio() const override;

    // True iff any tap has non-zero imaginary gain, so the analytic-pair
    // path is active. Diagnostics / model logging.
    bool needs_hilbert() const { return needs_hilbert_; }
    size_t hilbert_group_delay_samples() const {
        return hilbert_.group_delay_samples();
    }

private:
    struct ResolvedTap {
        size_t delta_samples;  // integer delay at source rate
        float  gain;           // channel_gain * tap_gain * cos(phase_rad)
        float  gain_imag;      // channel_gain * tap_gain * sin(phase_rad);
                               // any non-zero value enables the Hilbert path
                               // for the whole renderer.
    };

    // Bulk Farrow output -> PairBuffer at integer (source_pos + tap_delta).
    // Also handles the complex/Hilbert path.
    void scatter_taps(size_t source_position_start,
                      const float* farrow_out, size_t n,
                      PairBuffer& pair_buffer);

    dsp::FarrowResampler     resampler_;
    dsp::HilbertFilter       hilbert_;   // shared by all complex taps
    std::vector<ResolvedTap> taps_;

    size_t   base_delay_samples_    = 0;
    size_t   max_tap_delta_samples_ = 0;
    uint32_t sample_rate_           = 500'000;

    // Doppler inputs (copied from Params).
    float    clock_offset_ppm_         = 0.0f;
    float    sound_speed_m_s_          = 1500.0f;
    float    velocity_radial_m_s_      = 0.0f;
    float    acceleration_radial_m_s2_ = 0.0f;

    // Real-only channels skip the Hilbert pair and its group-delay offset.
    bool     needs_hilbert_ = false;

    // Per-message state (reset by on_message_start).
    size_t   source_samples_processed_ = 0;

    // Pre-allocated scratch.
    std::vector<float> resample_buf_;      // Farrow output, one batch
    std::vector<float> gained_buf_;        // farrow_out * tap.gain
    std::vector<float> hilbert_buf_;       // sized only when needs_hilbert_
    std::vector<float> delayed_real_buf_;  // sized only when needs_hilbert_
};

} // namespace openCREST
