#include "channel/channel.hpp"
#include "channel/geometric_scene.hpp"
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

double Channel::current_doppler_ratio() const {
    const double base_ratio = 1.0 +
        static_cast<double>(config_.clock_offset_ppm) * 1e-6;

    // Geometric mode: range-rate Doppler comes from time-varying tap deltas;
    // the Farrow ratio carries only the crystal-clock offset.
    if (geometric_) return base_ratio;

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
    , geometric_(config.mode == ChannelMode::Geometric)
{
    // Geometric mode: the acoustic chain (spreading + Thorp) comes from
    // GeometricScene per-block. Only the AFE-electrical chain and the
    // additive gain_db trim / receive boost multiply on top.
    if (geometric_) {
        const double afe_chain_db = dsp::compute_channel_afe_chain_gain_db(
            source_cal, receiver_cal,
            source_transducer, receiver_transducer,
            config_.rx_atten_idx);
        const double total_db = afe_chain_db
            + static_cast<double>(config_.gain_db)
            + static_cast<double>(receive_boost_db);
        afe_chain_gain_      = static_cast<float>(std::pow(10.0, total_db / 20.0));
        source_center_fc_hz_ = source_cal.center_freq_hz;
        saltwater_           = config_.saltwater;

        EnvironmentConfig env;
        env.sound_speed_m_s  = config_.sound_speed_m_s;
        env.saltwater        = config_.saltwater;
        env.spreading_factor = config_.spreading_factor;
        scene_ = std::make_unique<GeometricScene>(config_.geometry, env);

        // Anchor base_delay at the shortest enabled path at r_min so every
        // tap's excess delta_samples stays >= 0 across the R envelope.
        std::array<PathTap, 5> paths_rmin{};
        const std::size_t n_rmin = scene_->compute_paths(
            config_.geometry.r_min_m, paths_rmin);
        if (n_rmin == 0) {
            throw std::invalid_argument(
                "Channel: geometric mode requires at least one enabled path");
        }
        r_min_anchor_len_m_ = paths_rmin[0].length_m;
        const double sound_speed = clamp_sound_speed(config_.sound_speed_m_s);
        base_delay_samples_ = static_cast<size_t>(
            std::round(static_cast<double>(r_min_anchor_len_m_) /
                       sound_speed * sample_rate_));

        // Worst-case tap excess: longest path at r_max minus the r_min
        // anchor. Drives PairBuffer sizing.
        std::array<PathTap, 5> paths_rmax{};
        const std::size_t n_rmax = scene_->compute_paths(
            config_.geometry.r_max_m, paths_rmax);
        if (n_rmax == 0) {
            throw std::invalid_argument(
                "Channel: geometric mode requires at least one enabled path "
                "at r_max");
        }
        const float worst_len = paths_rmax[n_rmax - 1].length_m;
        const float worst_excess_s = std::max(
            0.0f, (worst_len - r_min_anchor_len_m_) /
                  static_cast<float>(sound_speed));
        max_tap_delta_samples_ = static_cast<size_t>(
            std::round(worst_excess_s * sample_rate_));

        // Path count is stable across blocks; size once using max(n_rmin,
        // n_rmax) for safety.
        const std::size_t tap_count = std::max(n_rmin, n_rmax);
        taps_.assign           (tap_count, ResolvedTap{0.0, 0.0f, 0.0f});
        geom_taps_start_.assign(tap_count, ResolvedTap{0.0, 0.0f, 0.0f});
        geom_taps_end_  .assign(tap_count, ResolvedTap{0.0, 0.0f, 0.0f});
        tap_states_     .assign(tap_count, dsp::TapState{});

        // SourceDelayLine sizing. Worst-case read-back behind the producer
        // is the longest enabled path's excess at the worst R in
        // [r_min, r_max] (strong reflections can put it at r_min, so check
        // both). Adds Catmull-Rom margin (4), clock-drift margin (one
        // block at configured PPM), and PROCESSING_BLOCK_SIZE for samples
        // written this block but not yet read.
        const double rmin_worst_excess_s = std::max(0.0,
            (static_cast<double>(paths_rmin[n_rmin - 1].length_m) -
             static_cast<double>(r_min_anchor_len_m_)) / sound_speed);
        const double rmax_worst_excess_s = std::max(0.0,
            (static_cast<double>(worst_len) -
             static_cast<double>(r_min_anchor_len_m_)) / sound_speed);
        const size_t rmin_worst_samples = static_cast<size_t>(
            std::ceil(rmin_worst_excess_s * sample_rate_));
        const size_t rmax_worst_samples = static_cast<size_t>(
            std::ceil(rmax_worst_excess_s * sample_rate_));
        const size_t worst_excess_samples =
            std::max(rmin_worst_samples, rmax_worst_samples);
        constexpr size_t catmull_rom_margin = 4;
        const double clock_drift_per_block =
            static_cast<double>(PROCESSING_BLOCK_SIZE) *
            std::abs(static_cast<double>(config_.clock_offset_ppm)) * 1e-6;
        const size_t clock_drift_margin =
            static_cast<size_t>(std::ceil(clock_drift_per_block)) + 1;
        const size_t sdl_min_capacity = worst_excess_samples
            + catmull_rom_margin + clock_drift_margin + PROCESSING_BLOCK_SIZE;
        source_delay_line_.resize(sdl_min_capacity);

        per_tap_scratch_.assign(PROCESSING_BLOCK_SIZE, 0.0f);

        spdlog::info(
            "Channel {} → {}: mode=geometric, AFE_dB={:.2f}, "
            "fc={:.1f}kHz, base_delay={} (anchor at r_min={:.1f}m direct={:.2f}m), "
            "max_tap_delta={} (worst-path at r_max={:.1f}m), "
            "source_delay_line_capacity={}",
            config_.from_modem, config_.to_modem,
            total_db, source_center_fc_hz_ / 1000.0,
            base_delay_samples_, config_.geometry.r_min_m, r_min_anchor_len_m_,
            max_tap_delta_samples_, config_.geometry.r_max_m,
            source_delay_line_.capacity());

        // Geometric mode bypasses scatter_taps(); these scratches are
        // unused but cheap to keep allocated.
        resample_buf_.assign(PROCESSING_BLOCK_SIZE, 0.0f);
        gained_buf_  .assign(PROCESSING_BLOCK_SIZE, 0.0f);
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

    if (geometric_) {
        // Snapshot R_0 and populate taps_ at t=0; process() refreshes
        // start/end taps each block.
        initial_range_m_ = (config_.initial_range_m > 0.0f)
            ? config_.initial_range_m : config_.range_m;
        recompute_geometric_taps(initial_range_m_);

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

void Channel::recompute_geometric_taps_into(
    float range_m, std::vector<ResolvedTap>& dst) const {
    if (!scene_) return;

    // Clamp to the configured envelope rather than throwing — sized
    // PairBuffer / SourceDelayLine assume reads stay in that window.
    const float r_clamped = std::clamp(range_m,
        config_.geometry.r_min_m, config_.geometry.r_max_m);

    std::array<PathTap, 5> paths{};
    const std::size_t n = scene_->compute_paths(r_clamped, paths);

    // Path count is stable across blocks; resize defensively in case the
    // scene's enable flags change at runtime.
    if (dst.size() != n) dst.assign(n, ResolvedTap{0.0, 0.0f, 0.0f});

    for (std::size_t i = 0; i < n; ++i) {
        const auto rp = scene_->resolve(paths[i], r_min_anchor_len_m_,
                                         source_center_fc_hz_,
                                         saltwater_,
                                         static_cast<float>(sample_rate_));
        dst[i].delta_samples_frac = rp.delta_samples_frac;
        dst[i].gain               = rp.gain_linear * afe_chain_gain_;
        dst[i].gain_imag          = 0.0f;
    }
}

void Channel::recompute_geometric_taps(float range_m) {
    recompute_geometric_taps_into(range_m, taps_);
}

float Channel::range_at_source_time(double t_seconds) const {
    const double v = static_cast<double>(config_.velocity_radial_m_s);
    const double a = static_cast<double>(config_.acceleration_radial_m_s2);
    return static_cast<float>(
        static_cast<double>(initial_range_m_) + v * t_seconds + 0.5 * a * t_seconds * t_seconds);
}

void Channel::on_message_end(PairBuffer& pair_buffer) {
    if (geometric_) {
        drain_geometric_tail(pair_buffer);
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

    if (geometric_) {
        return process_geometric(samples, count, pair_buffer);
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

size_t Channel::process_geometric(const float* samples, size_t count,
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

        recompute_geometric_taps_into(
            range_at_source_time(t_start), geom_taps_start_);
        recompute_geometric_taps_into(
            range_at_source_time(t_end),   geom_taps_end_);

        // Defensive: path count is stable across endpoints by construction.
        const size_t n_taps = std::min(geom_taps_start_.size(),
                                        geom_taps_end_.size());

        // Bake per-block clock drift into each tap's end-delay so the
        // PerTapFarrow inner loop stays uniform.
        const double clock_drift_per_block =
            static_cast<double>(n_out) * delta_clock;

        for (size_t k = 0; k < n_taps; ++k) {
            tap_states_[k].tap_delay_samples_at_block_start =
                geom_taps_start_[k].delta_samples_frac;
            tap_states_[k].tap_delay_samples_at_block_end =
                geom_taps_end_[k].delta_samples_frac
                - clock_drift_per_block;
            tap_states_[k].amplitude_at_block_start = geom_taps_start_[k].gain;
            tap_states_[k].amplitude_at_block_end   = geom_taps_end_[k].gain;

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

void Channel::drain_geometric_tail(PairBuffer& pair_buffer) {
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
        process_geometric(zeros.data(), chunk, pair_buffer);
        remaining -= chunk;
    }
}

size_t Channel::input_needed_for_batch() const {
    if (geometric_) {
        // No bulk Farrow in geometric mode: input-needed is exactly one block.
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
