#pragma once
#include <cstddef>
#include "dsp/source_delay_line.hpp"

namespace openCREST::dsp {

// State of one multipath tap over the duration of one output block.
//
// `tap_delay_samples_at_block_*` are in source-rate sample units
// (the same units as SourceDelayLine::producer_position). They are
// fractional. Linearly evolving the delay across the block is what
// creates path-specific Doppler — the read position advances at a
// rate other than 1 source sample per output sample.
//
// `amplitude_at_block_*` are the per-tap real-valued amplitude
// (path-loss × tap-gain × AFE chain). Lerp'd across the block so
// smooth range changes (e.g. R(t)) modulate the receiver level
// continuously rather than stepping every block.
//
// Future extension (Session H+): a complex amplitude pair could be
// added here to model surface-wave-induced phase modulation without
// full ray tracing off the wave field. Keep this struct trivially
// extensible — do not introduce fields beyond what current geometric
// mode needs.
struct TapState {
    double tap_delay_samples_at_block_start = 0.0;
    double tap_delay_samples_at_block_end   = 0.0;
    float  amplitude_at_block_start         = 0.0f;
    float  amplitude_at_block_end           = 0.0f;
};

// Output-driven per-tap interpolator.
//
// produce() writes exactly `n_out` samples into `out`, OVERWRITING any
// prior contents. The caller (Channel) then scatter-adds `out` into
// the destination PairBuffer — additive across taps so each receiver-
// time slot accumulates contributions from every multipath arrival.
//
// PerTapFarrow has no per-instance state; all timing lives in the
// SourceDelayLine (one per channel) and the TapState (one pair per tap
// per block).
class PerTapFarrow {
public:
    // For each output sample i ∈ [0, n_out):
    //   α = i / n_out
    //   tap_delay = lerp(start, end, α)
    //   pos       = out_pos_start + i − tap_delay
    //   amp       = lerp(amp_start, amp_end, α)
    //   out[i]    = amp · src.read_at(pos)
    //
    // out_pos_start is in source-rate sample units. Clock-offset rate
    // conversion is folded into TapState by the caller (Channel)
    // before invoking produce(), so this loop stays uniform.
    //
    // No-op when n_out == 0.
    static void produce(const SourceDelayLine& src,
                        const TapState&        tap,
                        double                 out_pos_start,
                        size_t                 n_out,
                        float*                 out);
};

} // namespace openCREST::dsp
