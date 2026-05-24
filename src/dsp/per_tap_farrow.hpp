#pragma once
#include <cstddef>
#include "dsp/source_delay_line.hpp"

namespace openCREST::dsp {

// State of one multipath tap over the duration of one output block.
//
// `tap_delay_samples_at_block_*` are fractional source-rate sample units
// (same units as SourceDelayLine::producer_position). Linearly evolving
// the delay across the block creates path-specific Doppler — the read
// position advances at a rate other than 1 source sample per output
// sample.
//
// `amplitude_at_block_*` are the per-tap real amplitude
// (path-loss × tap-gain × AFE chain). Lerp'd across the block so smooth
// range changes (R(t)) modulate the receiver level continuously.
struct TapState {
    double tap_delay_samples_at_block_start = 0.0;
    double tap_delay_samples_at_block_end   = 0.0;
    float  amplitude_at_block_start         = 0.0f;
    float  amplitude_at_block_end           = 0.0f;
};

// Output-driven per-tap interpolator. Stateless.
//
// produce() writes exactly `n_out` samples into `out`, OVERWRITING any
// prior contents. The caller (Channel) then scatter-adds `out` into the
// destination PairBuffer — additive across taps so each receiver-time
// slot accumulates contributions from every multipath arrival.
class PerTapFarrow {
public:
    // For each output sample i ∈ [0, n_out):
    //   α = i / n_out
    //   tap_delay = lerp(start, end, α)
    //   pos       = out_pos_start + i − tap_delay
    //   amp       = lerp(amp_start, amp_end, α)
    //   out[i]    = amp · src.read_at(pos)
    //
    // out_pos_start is in source-rate sample units. Any clock-offset rate
    // conversion is folded into TapState by the caller before calling
    // produce(), so this loop stays uniform. No-op when n_out == 0.
    static void produce(const SourceDelayLine& src,
                        const TapState&        tap,
                        double                 out_pos_start,
                        size_t                 n_out,
                        float*                 out);
};

} // namespace openCREST::dsp
