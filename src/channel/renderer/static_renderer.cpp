#include "channel/renderer/static_renderer.hpp"
#include "core/constants.hpp"

#include <algorithm>
#include <cmath>

namespace openCREST {

namespace {

// Zero samples fed to Farrow at message end to drain its 4-sample
// interpolation history so trailing samples decay cleanly to zero.
constexpr size_t kFarrowDrainZeros = 4;

double clamp_sound_speed(float configured) {
    return (configured > 0.0f) ? static_cast<double>(configured) : 1500.0;
}

} // namespace

StaticRenderer::StaticRenderer(Params params)
    : base_delay_samples_(params.base_delay_samples)
    , sample_rate_(params.sample_rate)
    , clock_offset_ppm_(params.clock_offset_ppm)
    , sound_speed_m_s_(params.sound_speed_m_s)
    , velocity_radial_m_s_(params.velocity_radial_m_s)
    , acceleration_radial_m_s2_(params.acceleration_radial_m_s2)
{
    const float channel_gain = params.channel_gain_linear;

    if (params.taps.empty()) {
        // Direct path, unity tap gain.
        taps_.push_back({0, channel_gain, 0.0f});
    } else {
        taps_.reserve(params.taps.size());
        for (const auto& t : params.taps) {
            ResolvedTap rt{};
            rt.delta_samples = static_cast<size_t>(
                std::round(static_cast<double>(t.delay_s) * sample_rate_));
            rt.gain      = t.gain_linear * std::cos(t.phase_rad) * channel_gain;
            rt.gain_imag = t.gain_linear * std::sin(t.phase_rad) * channel_gain;
            if (std::abs(rt.gain_imag) > 0.0f) needs_hilbert_ = true;
            taps_.push_back(rt);
            max_tap_delta_samples_ =
                std::max(max_tap_delta_samples_, rt.delta_samples);
        }
    }

    // Initial Doppler ratio; refreshed every process() call.
    resampler_.set_ratio(current_doppler_ratio());

    // Pre-allocated scratch — one Farrow-output batch.
    resample_buf_.assign(PROCESSING_BLOCK_SIZE, 0.0f);
    gained_buf_  .assign(PROCESSING_BLOCK_SIZE, 0.0f);
    if (needs_hilbert_) {
        hilbert_buf_     .assign(PROCESSING_BLOCK_SIZE, 0.0f);
        delayed_real_buf_.assign(PROCESSING_BLOCK_SIZE, 0.0f);
    }
}

double StaticRenderer::current_doppler_ratio() const {
    const double base_ratio =
        1.0 + static_cast<double>(clock_offset_ppm_) * 1e-6;

    const double sound_speed = clamp_sound_speed(sound_speed_m_s_);
    const double t           = (sample_rate_ > 0)
        ? static_cast<double>(source_samples_processed_) /
          static_cast<double>(sample_rate_)
        : 0.0;
    const double v = static_cast<double>(velocity_radial_m_s_)
                   + static_cast<double>(acceleration_radial_m_s2_) * t;
    return base_ratio * (1.0 + v / sound_speed);
}

void StaticRenderer::on_message_start() {
    resampler_.reset();
    if (needs_hilbert_) hilbert_.reset();
    source_samples_processed_ = 0;
    resampler_.set_ratio(current_doppler_ratio());
}

size_t StaticRenderer::process(const float* samples, size_t count,
                               PairBuffer& pair_buffer) {
    if (count == 0) return 0;

    // Refresh ratio from intra-message elapsed time so radial acceleration
    // smoothly modulates Doppler. No-op for zero acceleration.
    resampler_.set_ratio(current_doppler_ratio());

    size_t total_consumed = 0;

    while (total_consumed < count) {
        const float* in    = samples + total_consumed;
        const size_t avail = count - total_consumed;

        const auto result = resampler_.process(
            in, avail,
            resample_buf_.data(), resample_buf_.size());

        if (result.input_consumed == 0 && result.output_produced == 0) {
            // No forward progress possible.
            break;
        }

        if (result.output_produced > 0) {
            scatter_taps(source_samples_processed_,
                         resample_buf_.data(), result.output_produced,
                         pair_buffer);
            source_samples_processed_ += result.output_produced;
            pair_buffer.commit_source_progress(source_samples_processed_);
        }

        total_consumed += result.input_consumed;

        // Farrow stalled waiting for more input; caller supplies more next call.
        if (result.output_produced < resample_buf_.size()) break;
    }

    return total_consumed;
}

void StaticRenderer::on_message_end(PairBuffer& pair_buffer) {
    // Drain trailing FIR histories with zero input so the last samples decay
    // cleanly. Farrow's cubic Hermite needs ~4 zeros; the Hilbert filter
    // (when active) needs `group_delay` zeros. Use whichever is larger.
    const size_t drain = needs_hilbert_
        ? std::max<size_t>(kFarrowDrainZeros, hilbert_.group_delay_samples())
        : kFarrowDrainZeros;
    std::vector<float> zeros(drain, 0.0f);
    process(zeros.data(), drain, pair_buffer);

    // Expose the multipath tail at receiver positions
    // [S, S + max_tap_delta_samples_).
    pair_buffer.commit_extra(max_tap_delta_samples_);
}

size_t StaticRenderer::input_needed_for_batch() const {
    return resampler_.input_needed(resample_buf_.size());
}

void StaticRenderer::scatter_taps(size_t source_position_start,
                                  const float* farrow_out, size_t n,
                                  PairBuffer& pair_buffer) {
    if (!needs_hilbert_) {
        // Real-only taps: no Hilbert pair, no group-delay offset.
        for (const auto& tap : taps_) {
            for (size_t i = 0; i < n; ++i) {
                gained_buf_[i] = farrow_out[i] * tap.gain;
            }
            pair_buffer.scatter_add(source_position_start + tap.delta_samples,
                                    gained_buf_.data(), n);
        }
        return;
    }

    // Complex taps: project Farrow output onto the analytic pair
    // {delayed_real, hilbert_out} and apply each tap's complex gain as
    //   y = delayed_real * cos(phi)*g - hilbert_out * sin(phi)*g
    //     = Re{(g*e^(j*phi)) * (x + j*x_hat)}
    // The Hilbert filter time-aligns the two sub-outputs so (gain, gain_imag)
    // act on the same instant.
    hilbert_.process(farrow_out,
                     delayed_real_buf_.data(),
                     hilbert_buf_.data(), n);

    for (const auto& tap : taps_) {
        for (size_t i = 0; i < n; ++i) {
            gained_buf_[i] = delayed_real_buf_[i] * tap.gain
                           - hilbert_buf_[i]      * tap.gain_imag;
        }
        pair_buffer.scatter_add(source_position_start + tap.delta_samples,
                                gained_buf_.data(), n);
    }
}

} // namespace openCREST
