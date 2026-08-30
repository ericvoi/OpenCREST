#include "channel/model/static/static_model.hpp"

#include "channel/renderer/static_renderer.hpp"
#include "dsp/physical_gain.hpp"

#include <cmath>
#include <memory>

#include <spdlog/spdlog.h>

namespace openCREST {

std::unique_ptr<ChannelRenderer> make_static_renderer(
    const ChannelConfig& config, const ChannelBuildContext& ctx) {

    // Physically-derived per-channel scalar: path loss is computed at the
    // source modem's centre frequency; the result is the dB chain from source
    // ADC sample to receiver DAC sample. config.gain_db is an additive trim on
    // top. receive_boost_db is the per-receiver auto-boost applied when the
    // natural Wenz PSD at fc_rx sits below the AFE noise floor + min_margin —
    // every TX channel feeding that receiver is scaled up by the same dB so
    // SNR is preserved.
    const double physical_gain_db = dsp::compute_channel_physical_gain_db(
        ctx.source_cal, ctx.receiver_cal,
        ctx.source_transducer, ctx.receiver_transducer,
        config, config.rx_atten_idx);
    const double total_gain_db =
        physical_gain_db
        + static_cast<double>(config.gain_db)
        + static_cast<double>(ctx.receive_boost_db);
    const float channel_gain = static_cast<float>(
        std::pow(10.0, total_gain_db / 20.0));

    StaticRenderer::Params p;
    p.taps                     = config.multipath_taps;
    p.channel_gain_linear      = channel_gain;
    p.base_delay_samples       = bulk_propagation_delay_samples(config,
                                                                ctx.sample_rate);
    p.sample_rate              = ctx.sample_rate;
    p.clock_offset_ppm         = config.clock_offset_ppm;
    p.sound_speed_m_s          = config.sound_speed_m_s;
    p.velocity_radial_m_s      = config.velocity_radial_m_s;
    p.acceleration_radial_m_s2 = config.acceleration_radial_m_s2;

    auto renderer = std::make_unique<StaticRenderer>(std::move(p));

    // One-line chain summary for startup sanity-check. Clipping headroom
    // collapses fast above ~+20 dB; tune transducer values, attenuation index,
    // or the gain_db trim if total_gain_db is far from sensible.
    spdlog::info(
        "Channel {} → {}: mode=static, physical_gain_dB={:.2f} (range={:.1f}m, "
        "src_fc={:.1f}kHz, TVR={:.1f}, RVR={:.1f}, "
        "out_atten={:.1f}, in_atten[{}]={:.1f}, V_ref_adc={:.3f}, "
        "V_ref_dac={:.3f}); + gain_db trim={:.2f} + boost={:.2f} → "
        "total={:.2f} dB ({:.4g}×)",
        config.from_modem, config.to_modem,
        physical_gain_db, config.range_m,
        ctx.source_cal.center_freq_hz / 1000.0, ctx.source_transducer.tvr_db,
        ctx.receiver_transducer.rvr_db, ctx.source_cal.output_attenuation,
        static_cast<int>(config.rx_atten_idx),
        (config.rx_atten_idx < 2)
            ? ctx.receiver_cal.input_attenuation[config.rx_atten_idx] : 0.0f,
        ctx.source_cal.adc_vref_peak_volts, ctx.receiver_cal.dac_vref_peak_volts,
        config.gain_db, ctx.receive_boost_db, total_gain_db, channel_gain);

    if (renderer->needs_hilbert()) {
        spdlog::info(
            "Channel {} → {}: complex multipath taps detected — enabling "
            "Hilbert path (adds {} samples of group delay to every tap)",
            config.from_modem, config.to_modem,
            renderer->hilbert_group_delay_samples());
    }

    return renderer;
}

} // namespace openCREST
