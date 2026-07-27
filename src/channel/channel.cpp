#include "channel/channel.hpp"
#include "channel/geometric_tap_source.hpp"
#include "channel/replay_tap_source.hpp"
#include "core/constants.hpp"
#include "dsp/path_loss.hpp"
#include "dsp/physical_gain.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <spdlog/spdlog.h>

namespace openCREST {

// Zero samples fed to Farrow at message end to drain its 4-sample
// interpolation history so trailing samples decay cleanly to zero.
constexpr size_t kFarrowDrainZeros = 4;

namespace {

double clamp_sound_speed(float configured) {
    return (configured > 0.0f) ? static_cast<double>(configured) : 1500.0;
}

} // namespace

void Channel::init_read_style_buffers(size_t tap_count,
                                      size_t sdl_worst_excess_samples) {
    taps_start_buf_.assign(tap_count, SourcedTap{});
    taps_end_buf_  .assign(tap_count, SourcedTap{});
    tap_states_    .assign(tap_count, dsp::TapState{});

    // SourceDelayLine sizing. Worst-case read-back behind the producer
    // comes from the source; adds Catmull-Rom margin (4), clock-drift
    // margin (one block at configured PPM), and PROCESSING_BLOCK_SIZE for
    // samples written this block but not yet read.
    constexpr size_t catmull_rom_margin = 4;
    const double clock_drift_per_block =
        static_cast<double>(PROCESSING_BLOCK_SIZE) *
        std::abs(static_cast<double>(config_.clock_offset_ppm)) * 1e-6;
    const size_t clock_drift_margin =
        static_cast<size_t>(std::ceil(clock_drift_per_block)) + 1;
    const size_t sdl_min_capacity = sdl_worst_excess_samples
        + catmull_rom_margin + clock_drift_margin + PROCESSING_BLOCK_SIZE;
    source_delay_line_.resize(sdl_min_capacity);

    per_tap_scratch_.assign(PROCESSING_BLOCK_SIZE, 0.0f);

    // Read-style modes bypass scatter_taps(); these scratches are unused
    // but cheap to keep allocated.
    resample_buf_.assign(PROCESSING_BLOCK_SIZE, 0.0f);
    gained_buf_  .assign(PROCESSING_BLOCK_SIZE, 0.0f);
}

double Channel::current_doppler_ratio() const {
    const double base_ratio = 1.0 +
        static_cast<double>(config_.clock_offset_ppm) * 1e-6;

    // Read-style modes (geometric / replay): range-rate Doppler comes from
    // time-varying tap deltas; the Farrow ratio carries only the
    // crystal-clock offset.
    if (tap_source_) return base_ratio;

    const double sound_speed = clamp_sound_speed(config_.sound_speed_m_s);
    const double t           = (sample_rate_ > 0)
        ? static_cast<double>(source_samples_processed_) /
          static_cast<double>(sample_rate_)
        : 0.0;
    const double v = static_cast<double>(config_.velocity_radial_m_s)
                   + static_cast<double>(config_.acceleration_radial_m_s2) * t;
    return base_ratio * (1.0 + v / sound_speed);
}

Channel::Channel(const ChannelConfig&   config,
                 const CalibrationData& source_cal,
                 const CalibrationData& receiver_cal,
                 const TransducerSpec&  source_transducer,
                 const TransducerSpec&  receiver_transducer,
                 float                  receive_boost_db)
    : config_(config)
    , sample_rate_(source_cal.adc_sampling_rate)
{
    // Geometric mode: the acoustic chain (spreading + Thorp) comes from
    // GeometricTapSource per-block. Only the AFE-electrical chain and the
    // additive gain_db trim / receive boost multiply on top.
    if (config_.mode == ChannelMode::Geometric) {
        const double afe_chain_db = dsp::compute_channel_afe_chain_gain_db(
            source_cal, receiver_cal,
            source_transducer, receiver_transducer,
            config_.rx_atten_idx);
        const double total_db = afe_chain_db
            + static_cast<double>(config_.gain_db)
            + static_cast<double>(receive_boost_db);
        afe_chain_gain_ = static_cast<float>(std::pow(10.0, total_db / 20.0));

        auto source = std::make_unique<GeometricTapSource>(
            config_, source_cal.center_freq_hz, sample_rate_);
        base_delay_samples_    = source->base_delay_samples();
        max_tap_delta_samples_ = source->max_tap_delta_samples();

        init_read_style_buffers(source->tap_count_max(),
                                source->sdl_worst_excess_samples());

        spdlog::info(
            "Channel {} → {}: mode=geometric, AFE_dB={:.2f}, "
            "fc={:.1f}kHz, base_delay={} (anchor at r_min={:.1f}m direct={:.2f}m), "
            "max_tap_delta={} (worst-path at r_max={:.1f}m), "
            "source_delay_line_capacity={}",
            config_.from_modem, config_.to_modem,
            total_db, source_cal.center_freq_hz / 1000.0,
            base_delay_samples_, config_.geometry.r_min_m,
            source->r_min_anchor_len_m(),
            max_tap_delta_samples_, config_.geometry.r_max_m,
            source_delay_line_.capacity());

        tap_source_ = std::move(source);
        return;
    }

    // Replay mode: the acoustic path structure (delays, relative gains)
    // comes from the recorded trajectory file. Only the AFE-electrical
    // chain and the additive gain_db trim / receive boost multiply on top
    // — spreading/absorption are baked into the measured gains.
    if (config_.mode == ChannelMode::Replay) {
        const double afe_chain_db = dsp::compute_channel_afe_chain_gain_db(
            source_cal, receiver_cal,
            source_transducer, receiver_transducer,
            config_.rx_atten_idx);
        const double total_db = afe_chain_db
            + static_cast<double>(config_.gain_db)
            + static_cast<double>(receive_boost_db);
        afe_chain_gain_ = static_cast<float>(std::pow(10.0, total_db / 20.0));

        // Init-time file I/O; a malformed/missing file fails scenario
        // startup with TapTrajectoryError.
        auto source = std::make_unique<ReplayTapSource>(
            TapTrajectory::load(config_.replay.trajectory_path),
            config_.replay, sample_rate_,
            config_.from_modem + " → " + config_.to_modem);
        max_tap_delta_samples_ = source->max_tap_delta_samples();

        // Base propagation delay as in static mode: propagation_delay_s
        // overrides the range-derived delay (trajectory delays are excess
        // over this).
        const double sound_speed = clamp_sound_speed(config_.sound_speed_m_s);
        const double base_delay_seconds =
            (config_.propagation_delay_s >= 0.0f)
                ? static_cast<double>(config_.propagation_delay_s)
                : static_cast<double>(config_.range_m) / sound_speed;
        base_delay_samples_ = static_cast<size_t>(
            std::round(base_delay_seconds * sample_rate_));

        init_read_style_buffers(source->tap_count_max(),
                                max_tap_delta_samples_);

        spdlog::info(
            "Channel {} → {}: mode=replay, AFE_dB={:.2f}, file='{}' "
            "(taps={}, dt={:.1f}ms, duration={:.2f}s, fc_meas={:.1f}kHz), "
            "base_delay={}, max_tap_delta={}, source_delay_line_capacity={}",
            config_.from_modem, config_.to_modem,
            total_db, config_.replay.trajectory_path,
            source->tap_count_max(), source->dt_s() * 1000.0,
            source->duration_s(), source->fc_meas_hz() / 1000.0,
            base_delay_samples_, max_tap_delta_samples_,
            source_delay_line_.capacity());

        tap_source_ = std::move(source);
        return;
    }

    // Static mode. Physically-derived per-channel scalar: path loss is
    // computed at the source modem's center frequency; the result is the
    // dB chain from source ADC sample to receiver DAC sample.
    // config_.gain_db is an additive trim on top. `receive_boost_db` is
    // the per-receiver auto-boost applied when natural Wenz PSD at fc_rx
    // sits below the AFE noise floor + min_margin — every TX channel
    // feeding that receiver is scaled up by the same dB so SNR is
    // preserved.
    const double physical_gain_db = dsp::compute_channel_physical_gain_db(
        source_cal, receiver_cal,
        source_transducer, receiver_transducer,
        config_, config_.rx_atten_idx);
    const double total_gain_db =
        physical_gain_db
        + static_cast<double>(config_.gain_db)
        + static_cast<double>(receive_boost_db);
    const float channel_gain = static_cast<float>(
        std::pow(10.0, total_gain_db / 20.0));

    // One-line chain summary for startup sanity-check. Clipping headroom
    // collapses fast above ~+20 dB; tune transducer values, attenuation
    // index, or gain_db trim if total_gain_db is far from sensible.
    spdlog::info(
        "Channel {} → {}: physical_gain_dB={:.2f} (range={:.1f}m, "
        "src_fc={:.1f}kHz, TVR={:.1f}, RVR={:.1f}, "
        "out_atten={:.1f}, in_atten[{}]={:.1f}, V_ref_adc={:.3f}, "
        "V_ref_dac={:.3f}); + gain_db trim={:.2f} + boost={:.2f} → "
        "total={:.2f} dB ({:.4g}×)",
        config_.from_modem, config_.to_modem,
        physical_gain_db, config_.range_m,
        source_cal.center_freq_hz / 1000.0, source_transducer.tvr_db,
        receiver_transducer.rvr_db, source_cal.output_attenuation,
        static_cast<int>(config_.rx_atten_idx),
        (config_.rx_atten_idx < 2)
            ? receiver_cal.input_attenuation[config_.rx_atten_idx] : 0.0f,
        source_cal.adc_vref_peak_volts, receiver_cal.dac_vref_peak_volts,
        config_.gain_db, receive_boost_db, total_gain_db, channel_gain);

    if (config_.multipath_taps.empty()) {
        // Direct path, unity tap gain.
        ResolvedTap rt{};
        rt.delta_samples_frac = 0.0;
        rt.gain               = channel_gain;
        rt.gain_imag          = 0.0f;
        taps_.push_back(rt);
    } else {
        taps_.reserve(config_.multipath_taps.size());
        for (const auto& t : config_.multipath_taps) {
            ResolvedTap rt{};
            const size_t delta_int = static_cast<size_t>(
                std::round(static_cast<double>(t.delay_s) * sample_rate_));
            rt.delta_samples_frac = static_cast<double>(delta_int);
            rt.gain      = t.gain_linear * std::cos(t.phase_rad) * channel_gain;
            rt.gain_imag = t.gain_linear * std::sin(t.phase_rad) * channel_gain;
            if (std::abs(rt.gain_imag) > 0.0f) needs_hilbert_ = true;
            taps_.push_back(rt);
            max_tap_delta_samples_ =
                std::max(max_tap_delta_samples_, delta_int);
        }
    }

    // Base propagation delay in samples. propagation_delay_s overrides
    // the range-derived delay for refracting / non-geometric ray paths.
    const double sound_speed = clamp_sound_speed(config_.sound_speed_m_s);
    const double base_delay_seconds =
        (config_.propagation_delay_s >= 0.0f)
            ? static_cast<double>(config_.propagation_delay_s)
            : static_cast<double>(config_.range_m) / sound_speed;
    base_delay_samples_ = static_cast<size_t>(
        std::round(base_delay_seconds * sample_rate_));

    // Initial Doppler ratio; refreshed every process() call.
    resampler_.set_ratio(current_doppler_ratio());

    // Pre-allocated scratch — one Farrow-output batch.
    resample_buf_.assign(PROCESSING_BLOCK_SIZE, 0.0f);
    gained_buf_  .assign(PROCESSING_BLOCK_SIZE, 0.0f);
    if (needs_hilbert_) {
        hilbert_buf_     .assign(PROCESSING_BLOCK_SIZE, 0.0f);
        delayed_real_buf_.assign(PROCESSING_BLOCK_SIZE, 0.0f);
        spdlog::info(
            "Channel {} → {}: complex multipath taps detected — enabling "
            "Hilbert path (adds {} samples of group delay to every tap)",
            config_.from_modem, config_.to_modem,
            hilbert_.group_delay_samples());
    }
}

void Channel::on_message_start(PairBuffer& pair_buffer,
                                size_t inter_message_gap_samples,
                                bool   absolute_first_origin) {
    resampler_.reset();
    if (needs_hilbert_) hilbert_.reset();
    source_samples_processed_ = 0;

    if (tap_source_) {
        // Pin the source's time origin (e.g. snapshot R_0); process()
        // refreshes start/end taps each block.
        tap_source_->on_message_start();

        // Clear the SourceDelayLine so initial reads through the
        // Catmull-Rom margin see zero, not leftover prior-message samples.
        source_delay_line_.clear();
        pair_buffer_out_cursor_   = 0;
        source_delay_line_cursor_ = 0.0;
    } else {
        resampler_.set_ratio(current_doppler_ratio());
    }

    pair_buffer.begin_message(inter_message_gap_samples, absolute_first_origin);
}

Channel::~Channel() = default;

void Channel::on_message_end(PairBuffer& pair_buffer) {
    if (tap_source_) {
        drain_tap_source_tail(pair_buffer);
        return;
    }

    // Drain trailing FIR histories with zero input so the last samples
    // decay cleanly. Farrow's cubic Hermite needs ~4 zeros; the Hilbert
    // filter (when active) needs `group_delay` zeros. Use whichever is
    // larger.
    const size_t drain = needs_hilbert_
        ? std::max<size_t>(kFarrowDrainZeros, hilbert_.group_delay_samples())
        : kFarrowDrainZeros;
    std::vector<float> zeros(drain, 0.0f);
    process(zeros.data(), drain, pair_buffer);

    // Expose the multipath tail at receiver positions
    // [S, S + max_tap_delta_samples_).
    pair_buffer.commit_extra(max_tap_delta_samples_);
}

size_t Channel::process(const float* samples, size_t count,
                        PairBuffer& pair_buffer) {
    if (count == 0) return 0;

    if (tap_source_) {
        return process_tap_source(samples, count, pair_buffer);
    }

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

size_t Channel::process_tap_source(const float* samples, size_t count,
                                   PairBuffer& pair_buffer) {
    // 1:1 source-rate write.
    source_delay_line_.write(samples, count);

    // Produce exactly `count` receiver-rate samples per call so the
    // SourceDelayLine producer/consumer stay in lockstep at nominal Fs;
    // clock_offset_ppm drift accumulates in source_delay_line_cursor_
    // across blocks, and the per-tap formula folds it into each tap's
    // end-delay below.
    const double delta_clock = static_cast<double>(config_.clock_offset_ppm) * 1e-6;
    const double Fs_recv     = (sample_rate_ > 0)
        ? static_cast<double>(sample_rate_) : 500'000.0;

    size_t produced = 0;
    while (produced < count) {
        const size_t n_out = std::min<size_t>(
            PROCESSING_BLOCK_SIZE, count - produced);

        const double t_start =
            static_cast<double>(pair_buffer_out_cursor_) / Fs_recv;
        const double t_end =
            static_cast<double>(pair_buffer_out_cursor_ + n_out) / Fs_recv;

        const size_t n_start = tap_source_->taps_at(
            t_start, taps_start_buf_.data(), taps_start_buf_.size());
        const size_t n_end   = tap_source_->taps_at(
            t_end,   taps_end_buf_.data(),   taps_end_buf_.size());

        // Defensive: tap count is stable across endpoints by contract.
        const size_t n_taps = std::min(n_start, n_end);

        // Bake per-block clock drift into each tap's end-delay so the
        // PerTapFarrow inner loop stays uniform.
        const double clock_drift_per_block =
            static_cast<double>(n_out) * delta_clock;

        for (size_t k = 0; k < n_taps; ++k) {
            tap_states_[k].tap_delay_samples_at_block_start =
                taps_start_buf_[k].delta_samples_frac;
            tap_states_[k].tap_delay_samples_at_block_end =
                taps_end_buf_[k].delta_samples_frac
                - clock_drift_per_block;
            tap_states_[k].amplitude_at_block_start =
                taps_start_buf_[k].gain * afe_chain_gain_;
            tap_states_[k].amplitude_at_block_end   =
                taps_end_buf_[k].gain * afe_chain_gain_;

            dsp::PerTapFarrow::produce(
                source_delay_line_,
                tap_states_[k],
                source_delay_line_cursor_,
                n_out,
                per_tap_scratch_.data());

            // Per-tap delay lives in the read position, not the write offset.
            pair_buffer.scatter_add(pair_buffer_out_cursor_,
                                     per_tap_scratch_.data(), n_out);
        }

        pair_buffer_out_cursor_   += n_out;
        source_delay_line_cursor_ +=
            static_cast<double>(n_out) * (1.0 + delta_clock);

        pair_buffer.commit_source_progress(pair_buffer_out_cursor_);

        produced += n_out;
    }

    // Always consumes every input sample (1:1 write).
    return count;
}

void Channel::drain_tap_source_tail(PairBuffer& pair_buffer) {
    if (max_tap_delta_samples_ == 0) return;

    // Feed zeros so per-tap Farrow reads post-message silence once the
    // read pointer catches up with the producer. The longest tap trails
    // the producer by up to max_tap_delta_samples_; producing that many
    // extra receiver-rate samples lets every tap fully decay.
    const size_t tail = max_tap_delta_samples_;
    static thread_local std::vector<float> zeros;
    if (zeros.size() < PROCESSING_BLOCK_SIZE) {
        zeros.assign(PROCESSING_BLOCK_SIZE, 0.0f);
    }

    size_t remaining = tail;
    while (remaining > 0) {
        const size_t chunk = std::min<size_t>(PROCESSING_BLOCK_SIZE, remaining);
        process_tap_source(zeros.data(), chunk, pair_buffer);
        remaining -= chunk;
    }
}

size_t Channel::input_needed_for_batch() const {
    if (tap_source_) {
        // No bulk Farrow in read-style modes: input-needed is exactly one
        // block.
        return PROCESSING_BLOCK_SIZE;
    }
    return resampler_.input_needed(resample_buf_.size());
}

void Channel::scatter_taps(size_t source_position_start,
                            const float* farrow_out, size_t n,
                            PairBuffer& pair_buffer) {
    if (!needs_hilbert_) {
        // Real-only taps: no Hilbert pair, no group-delay offset.
        for (const auto& tap : taps_) {
            const size_t delta = static_cast<size_t>(
                std::lround(tap.delta_samples_frac));
            for (size_t i = 0; i < n; ++i) {
                gained_buf_[i] = farrow_out[i] * tap.gain;
            }
            pair_buffer.scatter_add(source_position_start + delta,
                                     gained_buf_.data(), n);
        }
        return;
    }

    // Complex taps: project Farrow output onto the analytic pair
    // {delayed_real, hilbert_out} and apply each tap's complex gain as
    //   y = delayed_real * cos(phi)*g - hilbert_out * sin(phi)*g
    //     = Re{(g*e^(j*phi)) * (x + j*x_hat)}
    // The Hilbert filter time-aligns the two sub-outputs so (gain,
    // gain_imag) act on the same instant.
    hilbert_.process(farrow_out,
                     delayed_real_buf_.data(),
                     hilbert_buf_.data(), n);

    for (const auto& tap : taps_) {
        const size_t delta = static_cast<size_t>(
            std::lround(tap.delta_samples_frac));
        for (size_t i = 0; i < n; ++i) {
            gained_buf_[i] = delayed_real_buf_[i] * tap.gain
                            - hilbert_buf_[i]    * tap.gain_imag;
        }
        pair_buffer.scatter_add(source_position_start + delta,
                                 gained_buf_.data(), n);
    }
}

} // namespace openCREST
