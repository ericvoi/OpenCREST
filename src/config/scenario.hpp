#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "core/types.hpp"
#include "dsp/noise_generator.hpp"  // TonalSource

namespace openCREST {

// Acoustic transducer parameters.
// TVR: transmit voltage response (dB re 1 µPa/V @ 1 m).
// RVR: receive voltage response (dB re 1 V/µPa).
// Frequency-flat scalars evaluated at the modem center frequency.
struct TransducerSpec {
    float tvr_db = 0.0f;
    float rvr_db = 0.0f;
};

// Channel propagation model.
//   Static    — scenario-supplied flat multipath tap list.
//   Geometric — method-of-images scene; taps recomputed each block from R(t),
//               yielding closing-range Doppler from shrinking tap delays.
//   Replay    — measured tap trajectories from an .octt file; Doppler
//               emerges from the recorded time-varying delays.
enum class ChannelMode { Static, Geometric, Replay };

// Replay-mode parameters. The record does not advance between messages:
// each message maps intra-message time onto the recording starting at a
// per-message offset.
struct ReplayConfig {
    // Absolute path to the .octt trajectory file (loader resolves the YAML
    // value relative to the scenario file's directory).
    std::string trajectory_path;

    // Record time at which the first message starts. In fixed mode
    // (advance_per_message = false) every message restarts here.
    double      offset_s              = 0.0;

    // Advancing mode: each message continues where the previous one (and
    // its multipath-tail drain) ended.
    bool        advance_per_message   = false;

    // Advancing mode: when the remaining record is shorter than this,
    // wrap to record time 0 instead of starting near the end. 0 = never
    // wrap (a message running past the end is truncated with a warning).
    double      wrap_if_remaining_lt_s = 0.0;

    // Populated by ScenarioLoader from the .octt header (not parsed from
    // YAML) so ChannelEngine sizing needs no file I/O.
    double      max_delay_s           = 0.0;
    uint32_t    tap_count             = 0;
};

struct GeometricSceneConfig {
    float water_depth_m         = 0.0f;
    float source_depth_m        = 0.0f;
    float receiver_depth_m      = 0.0f;

    // Reflection coefficients. Pressure-release surface defaults to -1;
    // mid-mud bottom ~+0.5.
    float gamma_surface         = -1.0f;
    float gamma_bottom          =  0.5f;

    // Spreading exponent k (cylindrical 1.0 ↔ spherical 2.0; default 1.5).
    float spreading_exponent_k  =  1.5f;

    bool  enable_direct         = true;
    bool  enable_surface        = true;
    bool  enable_bottom         = true;
    bool  enable_surface_bottom = false;
    bool  enable_bottom_surface = false;

    // Maximum reflection order (bounces) in the image expansion. 2 (default)
    // gives the classic five paths above; 3 and 4 add the order-3
    // (surface-bottom-surface, bottom-surface-bottom) and order-4 paths,
    // deepening the reverberant tail. Clamped to [0, kMaxImageOrder].
    int   max_bounces           = 2;

    // Range envelope used to size the PairBuffer. When <= 0, the loader
    // fills them with R_0/2 and R_0*2. R(t) must stay within these bounds.
    float r_min_m               = -1.0f;
    float r_max_m               = -1.0f;
};

struct ModemConfig {
    std::string id;
    std::string usb_serial;
    std::string transducer_id;

    // Crystal tolerance in ppm. σ of a zero-mean normal distribution truncated
    // to ±clock_offset_ppm; each modem independently samples its actual offset
    // at scenario load. Zero disables sampling.
    float       clock_offset_ppm         = 0.0f;

    // Per-modem sampled offset (ppm), populated by ScenarioLoader. Drives the
    // channel Doppler ratio. Not parsed from YAML.
    float       actual_clock_offset_ppm  = 0.0f;

    float       velocity_radial_m_s      = 0.0f;
    float       acceleration_radial_m_s2 = 0.0f;
};

// Per-channel parameters. Fields populated by ScenarioLoader (from
// EnvironmentConfig and the source ModemConfig) so Channel only needs to
// inspect one struct.
struct ChannelConfig {
    std::string            from_modem;
    std::string            to_modem;
    float                  range_m                   = 150.0f;

    // Channel-level gain (dB) folded into every tap. Models RX transducer
    // + pre-amp gain that is bypassed in physical loopback, or compensates
    // spreading/absorption for self-loopback through the DAC. Tap gains in
    // YAML are then relative dB to the main tap.
    float                  gain_db                   = 0.0f;

    // Whether the line-of-sight (direct) ray exists. Default true: caller
    // includes a zero-delay tap; remaining tap delays are excess over the
    // direct arrival. Set false for shadow zones / blocked-LOS geometries
    // where only reflected rays arrive; multipath_taps must then be given
    // with a non-zero earliest delay.
    bool                   direct_los                = true;

    // Optional override of the direct-path propagation delay in seconds.
    // Negative = compute base_delay = range_m / sound_speed_m_s. Non-negative
    // is used verbatim (refracting deep-ocean rays where geometric range
    // does not predict travel time). range_m still drives path loss.
    float                  propagation_delay_s       = -1.0f;

    // Inherited from EnvironmentConfig. Path-loss frequency comes from the
    // source modem's calibration (CalibrationData::center_freq_hz);
    // heterogeneous-fc pairs are valid.
    float                  spreading_factor           = 2.0f;   // spherical default
    bool                   saltwater                  = true;
    float                  sound_speed_m_s            = 1500.0f; // for velocity ratio

    // Effective clock-offset for this directed channel, in ppm:
    //   = source.actual_clock_offset_ppm − receiver.actual_clock_offset_ppm
    // Loopback (source == receiver) is always 0.
    float                  clock_offset_ppm           = 0.0f;

    // From source ModemConfig (inherited by ScenarioLoader)
    float                  velocity_radial_m_s        = 0.0f;
    float                  acceleration_radial_m_s2   = 0.0f;

    // Index into receiver_cal.input_attenuation[] that the physical-gain
    // chain must undo — must match whichever pad the hardware has switched
    // in for this RX channel.
    //
    // Default 1 (the −63 dB pad) matches the OpenAquatix firmware, which
    // forces ATTENUATION_63DB during HIL operation regardless of host
    // requests. Index 0 (−93 dB) is only used briefly during the
    // calibration loopback tone.
    uint8_t                rx_atten_idx               = 1;

    std::vector<MultipathTap> multipath_taps;

    // Propagation model.
    //   Static    — use multipath_taps directly (default).
    //   Geometric — compute taps from `geometry` each block; multipath_taps
    //               must be empty.
    //   Replay    — render tap trajectories from `replay`; multipath_taps
    //               and geometry must be empty.
    ChannelMode            mode             = ChannelMode::Static;
    GeometricSceneConfig   geometry;        // populated when mode == Geometric
    ReplayConfig           replay;          // populated when mode == Replay

    // R_0 for geometric mode. Negative = fall back to range_m; otherwise
    // overrides the snapshot range at on_message_start so the scene moves
    // through a deliberate range envelope independent of range_m (which
    // still drives scenario-wide max-range sizing).
    float                  initial_range_m  = -1.0f;
};

struct NoiseConfig {
    int   wenz_sea_state            = 3;

    // Minimum dB by which the simulator's natural ambient PSD (Wenz at the
    // receiver's center frequency) must exceed the modem's measured AFE
    // PSD. When the natural value falls below `afe + min_margin_above_afe_db`,
    // every channel feeding that receiver is boosted by the deficit (and so
    // is the simulator noise, preserving SNR at the cost of clipping
    // headroom). Default 10 dB.
    float min_margin_above_afe_db   = 10.0f;

    // Short-circuit receiver-side ambient noise to zero. Tonals still pass
    // through verbatim unless their amplitude is also zero.
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
    // If <= 0, falls back to the largest channel range in the scenario.
    float       max_range_m       = 0.0f;

    // Longest single-message duration (seconds) the engine must buffer per
    // PairBuffer. In half-duplex loopback the entire message has to fit in
    // flight. Default 10 s comfortably covers realistic FH-BFSK / coded
    // payloads.
    float       max_message_duration_s = 10.0f;
};

struct LoggingConfig {
    bool        log_raw_tx        = false;
    bool        log_raw_rx        = false;
    bool        log_processed     = false;
    std::string output_directory  = ".";
    std::string file_format       = "wav";

    // Observability artifacts; defaults off, files land in `output_directory`.
    bool        log_processing_time_histogram = false;
    bool        log_message_events            = false;
    // Run-end summary JSON path. When empty, falls back to
    // `<output_directory>/<scenario_name>_summary.json`. Setting this also
    // enables summary emission.
    std::string run_summary_path;
};

struct ScenarioConfig {
    std::string       name;
    std::string       description;
    EnvironmentConfig environment;
    NoiseConfig       noise;
    // Transducer model library, keyed by id. Every modem's transducer_id
    // must reference an entry here.
    std::map<std::string, TransducerSpec> transducers;
    std::vector<ModemConfig>   modems;
    std::vector<ChannelConfig> channels;
    LoggingConfig     logging;

    // Seed for static stochastic parameter draws at scenario load
    // (currently: per-modem actual_clock_offset_ppm). Override in YAML when
    // sweeping draws.
    uint64_t          random_seed       = 12345;
};

} // namespace openCREST
