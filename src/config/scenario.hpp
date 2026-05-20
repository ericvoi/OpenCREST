#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "core/types.hpp"
#include "dsp/noise_generator.hpp"  // TonalSource

namespace openCREST {

// Acoustic transducer parameters. TVR is the transmit voltage response
// (dB re 1 µPa/V @ 1 m); RVR is the receive voltage response (dB re 1
// V/µPa). MVP ships frequency-flat values; the YAML schema is structured
// so a future FIR breakpoint table fits without rearrangement (the
// physical-gain math collapses to a single scalar at the modem center
// frequency for now).
struct TransducerSpec {
    float tvr_db = 0.0f;
    float rvr_db = 0.0f;
};

// Channel propagation model. Static = flat, scenario-supplied multipath tap
// list (legacy behavior). Geometric = method-of-images scene; taps are
// recomputed each processing block from R(t) so closing-range Doppler emerges
// naturally as the direct-path delta_samples shrinks.
enum class ChannelMode { Static, Geometric };

struct GeometricSceneConfig {
    float water_depth_m         = 0.0f;
    float source_depth_m        = 0.0f;
    float receiver_depth_m      = 0.0f;

    // Reflection coefficients. Pressure-release surface defaults to -1; a
    // mid-mud bottom around +0.5 is the paper's reference value.
    float gamma_surface         = -1.0f;
    float gamma_bottom          =  0.5f;

    // Spreading exponent k in eq.(2) (between cylindrical 1.0 and spherical
    // 2.0; the paper defaults to 1.5).
    float spreading_exponent_k  =  1.5f;

    bool  enable_direct         = true;
    bool  enable_surface        = true;
    bool  enable_bottom         = true;
    bool  enable_surface_bottom = false;
    bool  enable_bottom_surface = false;

    // Range envelope for PairBuffer sizing. When <= 0 the loader fills them
    // with R_0/2 and R_0*2 respectively. R(t) is asserted within these
    // bounds at runtime.
    float r_min_m               = -1.0f;
    float r_max_m               = -1.0f;
};

struct ModemConfig {
    std::string id;
    std::string usb_serial;
    std::string transducer_id;

    // Crystal tolerance, in ppm — interpreted as the σ of a zero-mean normal
    // distribution truncated to ±clock_offset_ppm. At scenario load each modem
    // independently draws its actual_clock_offset_ppm from this distribution,
    // so two modems with the same tolerance still see a non-zero relative
    // offset on the channel between them. A value of 0 disables sampling
    // (actual_clock_offset_ppm stays 0, like before).
    float       clock_offset_ppm         = 0.0f;

    // Sampled per-modem clock offset (ppm), populated by ScenarioLoader from
    // clock_offset_ppm + the scenario random_seed. Used as δ in the channel
    // Doppler ratio. Not parsed from YAML.
    float       actual_clock_offset_ppm  = 0.0f;

    float       velocity_radial_m_s      = 0.0f;
    float       acceleration_radial_m_s2 = 0.0f;
};

// Per-channel parameters.  Fields marked "from EnvironmentConfig" and
// "from source ModemConfig" are populated by ScenarioLoader so that Channel
// only needs to inspect one struct.
struct ChannelConfig {
    std::string            from_modem;
    std::string            to_modem;
    float                  range_m                   = 150.0f;

    // Channel-level gain (dB) folded into every tap. Use this to model the
    // RX transducer + pre-amp gain that is bypassed in physical loopback, or
    // to compensate spreading/absorption when running self-loopback through
    // the DAC. Tap gains in YAML can then be expressed cleanly as relative
    // dB to the main tap without needing per-tap path-loss compensation.
    float                  gain_db                   = 0.0f;

    // Whether the line-of-sight (direct) ray exists. Default true: the user
    // is expected to include a zero-delay tap representing the direct path,
    // and tap delays are excess over the direct arrival time. Set to false
    // for shadow zones or blocked-LOS geometries where only reflected rays
    // arrive — in that case multipath_taps must be specified explicitly and
    // typically the earliest tap has a non-zero delay.
    bool                   direct_los                = true;

    // Optional override of the direct-path propagation delay in seconds. When
    // negative (default), the channel computes base_delay = range_m /
    // sound_speed_m_s. When non-negative it is used verbatim — useful for
    // refracting deep-ocean rays where geometric range does not predict the
    // actual ray-path travel time. range_m still drives path loss.
    float                  propagation_delay_s       = -1.0f;

    // From EnvironmentConfig (inherited by ScenarioLoader). The path-loss
    // frequency for this channel comes from the source modem's calibration
    // (CalibrationData::center_freq_hz) — heterogeneous-fc pairs are valid,
    // and acoustic propagation depends on the radiated frequency.
    float                  spreading_factor           = 2.0f;   // spherical default
    bool                   saltwater                  = true;
    float                  sound_speed_m_s            = 1500.0f; // for velocity ratio

    // Effective clock-offset for this directed channel, in ppm:
    //   = source.actual_clock_offset_ppm − receiver.actual_clock_offset_ppm
    // Loopback (source == receiver) is therefore always 0 — a modem cannot
    // have a clock offset relative to itself. Populated by ScenarioLoader.
    float                  clock_offset_ppm           = 0.0f;

    // From source ModemConfig (inherited by ScenarioLoader)
    float                  velocity_radial_m_s        = 0.0f;
    float                  acceleration_radial_m_s2   = 0.0f;

    // Selects which entry of receiver_cal.input_attenuation[] is undone in
    // the physical-gain chain — the simulator must match whichever pad the
    // hardware switches in for this RX channel. MVP: a single fixed value
    // per channel. Live switching mid-scenario is post-MVP.
    //
    // Default 1 (the −63 dB pad) matches the OpenAquatix firmware, which
    // forces ATTENUATION_63DB during HIL operation regardless of host
    // requests. Index 0 (−93 dB) is only used briefly during the
    // calibration loopback tone.
    uint8_t                rx_atten_idx               = 1;

    std::vector<MultipathTap> multipath_taps;

    // Propagation model. Static = legacy flat multipath_taps list (the
    // default). Geometric = compute taps from `geometry` each block as R(t)
    // evolves; in that mode multipath_taps must be empty.
    ChannelMode            mode             = ChannelMode::Static;
    GeometricSceneConfig   geometry;        // populated when mode == Geometric

    // R_0 for geometric mode. Negative = "fall back to range_m"; otherwise
    // overrides the snapshot range at on_message_start so the scene moves
    // through a deliberate range envelope independent of range_m (which
    // continues to feed scenario-wide max-range sizing).
    float                  initial_range_m  = -1.0f;
};

struct NoiseConfig {
    int   wenz_sea_state            = 3;

    // Minimum dB by which the simulator's natural ambient PSD (Wenz at the
    // receiver's center frequency) must exceed the modem's measured AFE
    // PSD before the simulation is considered physically meaningful. When
    // the natural value falls below `afe + min_margin_above_afe_db`, every
    // channel feeding that receiver is boosted by the deficit (and so is
    // the simulator noise, preserving SNR at the cost of clipping
    // headroom). Default 10 dB.
    float min_margin_above_afe_db   = 10.0f;

    // When true, the receiver-side ambient noise is short-circuited to
    // zero. Useful for `loopback_identity` and unit tests where the
    // channel must be transparent. Tonals still pass through verbatim
    // unless their amplitude is also zero.
    bool  disable                   = false;

    bool  saltwater                 = true;
    std::vector<dsp::TonalSource> tonal_sources;
};

struct EnvironmentConfig {
    float       sound_speed_m_s   = 1500.0f;
    bool        saltwater         = true;
    std::string spreading_model   = "spherical"; // "spherical", "cylindrical", "hybrid"
    float       spreading_factor  = 2.0f;        // k in TL = k·log10(r)
    // Worst-case range used to size every PairBuffer in the ChannelEngine.
    // If <= 0, the loader/engine falls back to the largest channel range in
    // the scenario. Allows scenarios to budget memory for future longer
    // ranges without changing every channel.
    float       max_range_m       = 0.0f;

    // Longest single-message duration (seconds) the engine must buffer per
    // PairBuffer. The receiver only drains while the destination modem is
    // in RX state, so in half-duplex loopback (and when the receiver's
    // pull-thread runs less often than the source produces) the entire
    // message has to fit in flight. Default 10 s comfortably covers
    // realistic FH-BFSK / coded payloads. Increase for very long messages,
    // decrease to shrink memory at scale.
    float       max_message_duration_s = 10.0f;
};

struct LoggingConfig {
    bool        log_raw_tx        = false;
    bool        log_raw_rx        = false;
    bool        log_processed     = false;
    std::string output_directory  = ".";
    std::string file_format       = "wav";

    // Session D observability — defaults are off so existing scenarios
    // produce no new files. When enabled, artifacts land in
    // `output_directory`.
    bool        log_processing_time_histogram = false;
    bool        log_message_events            = false;
    // Absolute or relative path for the run-end summary JSON. When empty,
    // the writer falls back to `<output_directory>/<scenario_name>_summary.json`.
    // Setting this also enables summary emission.
    std::string run_summary_path;
};

struct ScenarioConfig {
    std::string       name;
    std::string       description;
    EnvironmentConfig environment;
    NoiseConfig       noise;
    // Transducer model library, keyed by id. Every modem's transducer_id
    // must reference an entry here. Phase A populates this from the
    // top-level `transducers:` YAML section; Phase B consumes it for the
    // physical-gain computation in Channel.
    std::map<std::string, TransducerSpec> transducers;
    std::vector<ModemConfig>   modems;
    std::vector<ChannelConfig> channels;
    LoggingConfig     logging;

    // Seed for any stochastic-but-static parameter draws done at scenario
    // load (currently: per-modem actual_clock_offset_ppm). Fixed default so
    // runs are reproducible; override in YAML when sweeping draws.
    uint64_t          random_seed       = 12345;
};

} // namespace openCREST
