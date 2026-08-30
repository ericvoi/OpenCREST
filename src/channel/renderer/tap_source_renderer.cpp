#include "channel/renderer/tap_source_renderer.hpp"
#include "core/constants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace openCREST {

TapSourceRenderer::TapSourceRenderer(Params params)
    : tap_source_(std::move(params.tap_source))
    , afe_chain_gain_(params.afe_chain_gain_linear)
    , base_delay_samples_(params.base_delay_samples)
    , sample_rate_(params.sample_rate)
    , clock_offset_ppm_(params.clock_offset_ppm)
{
    if (!tap_source_) {
        throw std::invalid_argument(
            "TapSourceRenderer: tap_source must not be null");
    }

    max_tap_delta_samples_ = tap_source_->max_tap_delta_samples();

    const size_t tap_count = tap_source_->tap_count_max();
    taps_start_buf_.assign(tap_count, SourcedTap{});
    taps_end_buf_  .assign(tap_count, SourcedTap{});
    tap_states_    .assign(tap_count, dsp::TapState{});

    // SourceDelayLine sizing. Worst-case read-back behind the producer comes
    // from the model; add Catmull-Rom margin (4), clock-drift margin (one
    // block at the configured PPM), and PROCESSING_BLOCK_SIZE for samples
    // written this block but not yet read.
    constexpr size_t catmull_rom_margin = 4;
    const double clock_drift_per_block =
        static_cast<double>(PROCESSING_BLOCK_SIZE) *
        std::abs(static_cast<double>(clock_offset_ppm_)) * 1e-6;
    const size_t clock_drift_margin =
        static_cast<size_t>(std::ceil(clock_drift_per_block)) + 1;
    const size_t sdl_min_capacity = params.sdl_worst_excess_samples
        + catmull_rom_margin + clock_drift_margin + PROCESSING_BLOCK_SIZE;
    source_delay_line_.resize(sdl_min_capacity);

    per_tap_scratch_.assign(PROCESSING_BLOCK_SIZE, 0.0f);
}

TapSourceRenderer::~TapSourceRenderer() = default;

void TapSourceRenderer::on_message_start() {
    // Pin the source's time origin (e.g. snapshot R_0); process() refreshes
    // start/end taps each block.
    tap_source_->on_message_start();

    // Clear the SourceDelayLine so initial reads through the Catmull-Rom
    // margin see zero, not leftover prior-message samples.
    source_delay_line_.clear();
    pair_buffer_out_cursor_   = 0;
    source_delay_line_cursor_ = 0.0;
}

size_t TapSourceRenderer::process(const float* samples, size_t count,
                                  PairBuffer& pair_buffer) {
    if (count == 0) return 0;

    // 1:1 source-rate write.
    source_delay_line_.write(samples, count);

    // Produce exactly `count` receiver-rate samples per call so the
    // SourceDelayLine producer/consumer stay in lockstep at nominal Fs;
    // clock_offset_ppm drift accumulates in source_delay_line_cursor_ across
    // blocks, and the per-tap formula folds it into each tap's end-delay
    // below.
    const double delta_clock =
        static_cast<double>(clock_offset_ppm_) * 1e-6;
    const double Fs_recv = (sample_rate_ > 0)
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

void TapSourceRenderer::on_message_end(PairBuffer& pair_buffer) {
    drain_tail(pair_buffer);
}

void TapSourceRenderer::drain_tail(PairBuffer& pair_buffer) {
    if (max_tap_delta_samples_ == 0) return;

    // Feed zeros so per-tap Farrow reads post-message silence once the read
    // pointer catches up with the producer. The longest tap trails the
    // producer by up to max_tap_delta_samples_; producing that many extra
    // receiver-rate samples lets every tap fully decay.
    static thread_local std::vector<float> zeros;
    if (zeros.size() < PROCESSING_BLOCK_SIZE) {
        zeros.assign(PROCESSING_BLOCK_SIZE, 0.0f);
    }

    size_t remaining = max_tap_delta_samples_;
    while (remaining > 0) {
        const size_t chunk = std::min<size_t>(PROCESSING_BLOCK_SIZE, remaining);
        process(zeros.data(), chunk, pair_buffer);
        remaining -= chunk;
    }
}

size_t TapSourceRenderer::input_needed_for_batch() const {
    // No bulk Farrow here: input-needed is exactly one block.
    return PROCESSING_BLOCK_SIZE;
}

} // namespace openCREST
