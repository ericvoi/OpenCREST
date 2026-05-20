#pragma once
#include "config/scenario.hpp"
#include <string>
#include <stdexcept>

namespace openCREST {

// Raised when a scenario YAML file fails validation.
class ScenarioLoadError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Load and validate ScenarioConfig from YAML.
//
// YAML schema overview:
//
//   name: "scenario name"
//   description: "..."          # optional
//
//   environment:
//     sound_speed_m_s: 1500.0   # optional, default 1500
//     saltwater: true            # optional, default true
//     spreading_model: spherical # optional: spherical|cylindrical|hybrid
//     # NOTE: center_freq_khz was removed; modem center frequency now comes
//     # from each modem's firmware-reported calibration data.
//
//   modems:
//     - id: modem_a
//       usb_serial: "ABC123"
//       transducer_id: "T-001"   # optional
//       clock_offset_ppm: 0.0    # optional
//       velocity_radial_m_s: 0.0 # optional
//
//   channels:
//     - from: modem_a
//       to: modem_b              # may equal from for loopback
//       range_m: 150.0
//       multipath_taps:          # optional
//         - delay_s: 0.0
//           gain_db: 0.0         # converted to linear amplitude
//           phase_deg: 0.0       # converted to radians
//
//   noise:                       # optional section
//     wenz_sea_state: 3
//     min_margin_above_afe_db: 10.0   # default; PSD margin above AFE floor
//     disable: false                  # short-circuit ambient noise
//     saltwater: true
//     tonal_sources:             # optional
//       - frequency_hz: 60.0
//         amplitude_linear: 0.01
//         bandwidth_hz: 0.0
//
//   logging:                     # optional section
//     log_raw_tx: false
//     log_raw_rx: false
//     log_processed: false
//     output_directory: "."
//     file_format: wav
class ScenarioLoader {
public:
    // Load from a YAML file path.
    static ScenarioConfig load(const std::string& filepath);

    // Load from a YAML string (used in unit tests to avoid filesystem access).
    static ScenarioConfig load_from_string(const std::string& yaml_text);
};

} // namespace openCREST
