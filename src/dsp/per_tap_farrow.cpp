#include "dsp/per_tap_farrow.hpp"

namespace openCREST::dsp {

void PerTapFarrow::produce(const SourceDelayLine& src,
                           const TapState&        tap,
                           double                 out_pos_start,
                           size_t                 n_out,
                           float*                 out) {
    if (n_out == 0) return;

    const double inv_n     = 1.0 / static_cast<double>(n_out);
    const double delay_d   = tap.tap_delay_samples_at_block_end
                           - tap.tap_delay_samples_at_block_start;
    const double amp_start = static_cast<double>(tap.amplitude_at_block_start);
    const double amp_d     = static_cast<double>(tap.amplitude_at_block_end)
                           - amp_start;

    for (size_t i = 0; i < n_out; ++i) {
        const double alpha    = static_cast<double>(i) * inv_n;
        const double tap_delay =
            tap.tap_delay_samples_at_block_start + alpha * delay_d;
        const double pos      = out_pos_start + static_cast<double>(i)
                              - tap_delay;
        const float  amp      = static_cast<float>(amp_start + alpha * amp_d);
        out[i] = amp * src.read_at(pos);
    }
}

} // namespace openCREST::dsp
