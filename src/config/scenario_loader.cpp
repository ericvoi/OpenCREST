#include "config/scenario_loader.hpp"
#include "core/constants.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstdint>
#include <random>
#include <set>
#include <sstream>

namespace openCREST {

namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

float spreading_factor_from_model(const std::string& model) {
    if (model == "cylindrical") return 1.0f;
    if (model == "hybrid")      return 1.5f;
    return 2.0f;  // spherical (default)
}

// Draw a sample from a normal distribution with σ = tolerance, rejecting
// values that fall outside [−tolerance, +tolerance]. With σ = tolerance the
// untruncated tails contribute ~32 % of mass, so rejection terminates fast
// in practice; we still cap retries to keep this bounded.
float draw_truncated_normal_ppm(float tolerance, std::mt19937_64& rng) {
    if (tolerance <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, tolerance);
    constexpr int kMaxAttempts = 1000;
    for (int i = 0; i < kMaxAttempts; ++i) {
        const float x = dist(rng);
        if (std::fabs(x) <= tolerance) return x;
    }
    // Should never happen with σ = tolerance (P(|x| > σ) ≈ 0.32, so 1000
    // consecutive rejects has probability ~10⁻⁴⁹⁵). Fall through with a
    // clamped value just in case.
    return 0.0f;
}

// Parse and validate the top-level structure.
ScenarioConfig parse(const YAML::Node& root, const std::string& source) {
    if (!root.IsMap()) {
        throw ScenarioLoadError(source + ": root node must be a YAML mapping");
    }

    ScenarioConfig cfg;

    // --- name / description ---
    if (!root["name"]) {
        throw ScenarioLoadError(source + ": missing required field 'name'");
    }
    cfg.name        = root["name"].as<std::string>();
    cfg.description = root["description"] ? root["description"].as<std::string>() : "";

    if (root["random_seed"]) {
        cfg.random_seed = root["random_seed"].as<uint64_t>();
    }

    // --- environment (optional) ---
    if (const YAML::Node& env = root["environment"]) {
        if (env["sound_speed_m_s"])
            cfg.environment.sound_speed_m_s = env["sound_speed_m_s"].as<float>();
        if (env["saltwater"])
            cfg.environment.saltwater = env["saltwater"].as<bool>();
        if (env["spreading_model"]) {
            cfg.environment.spreading_model  = env["spreading_model"].as<std::string>();
            cfg.environment.spreading_factor = spreading_factor_from_model(cfg.environment.spreading_model);
        }
        if (env["spreading_factor"])
            cfg.environment.spreading_factor = env["spreading_factor"].as<float>();
        if (env["center_freq_khz"]) {
            throw ScenarioLoadError(source +
                ": environment.center_freq_khz is no longer accepted — modem "
                "center frequency now comes from the firmware-reported "
                "calibration (CalibrationData::center_freq_hz). Remove this "
                "field from the scenario YAML.");
        }
        if (env["max_range_m"])
            cfg.environment.max_range_m = env["max_range_m"].as<float>();
        if (env["max_message_duration_s"]) {
            cfg.environment.max_message_duration_s =
                env["max_message_duration_s"].as<float>();
            if (cfg.environment.max_message_duration_s <= 0.0f) {
                throw ScenarioLoadError(source +
                    ": environment.max_message_duration_s must be > 0");
            }
        }
    }

    // --- transducers (required if any modem references one) ---
    // We parse this before modems so that modem-side validation can
    // verify every transducer_id resolves to a defined entry.
    if (const YAML::Node& tx_node = root["transducers"]) {
        if (!tx_node.IsMap()) {
            throw ScenarioLoadError(source +
                ": 'transducers' must be a mapping of id → {tvr_db, rvr_db}");
        }
        for (auto it = tx_node.begin(); it != tx_node.end(); ++it) {
            const std::string id = it->first.as<std::string>();
            const YAML::Node& spec = it->second;
            if (!spec.IsMap()) {
                throw ScenarioLoadError(source +
                    ": transducer '" + id + "' must be a mapping with tvr_db / rvr_db");
            }
            if (!spec["tvr_db"]) {
                throw ScenarioLoadError(source +
                    ": transducer '" + id + "' missing required field 'tvr_db'");
            }
            if (!spec["rvr_db"]) {
                throw ScenarioLoadError(source +
                    ": transducer '" + id + "' missing required field 'rvr_db'");
            }
            TransducerSpec ts;
            ts.tvr_db = spec["tvr_db"].as<float>();
            ts.rvr_db = spec["rvr_db"].as<float>();
            cfg.transducers.emplace(id, ts);
        }
    }

    // --- modems ---
    if (!root["modems"]) {
        throw ScenarioLoadError(source + ": missing required section 'modems'");
    }
    const YAML::Node& modems_node = root["modems"];
    if (!modems_node.IsSequence() || modems_node.size() == 0) {
        throw ScenarioLoadError(source + ": 'modems' must be a non-empty sequence");
    }
    if (modems_node.size() > MAX_MODEMS) {
        std::ostringstream oss;
        oss << source << ": 'modems' has " << modems_node.size()
            << " entries, max is " << MAX_MODEMS;
        throw ScenarioLoadError(oss.str());
    }

    std::set<std::string> modem_ids;
    for (std::size_t i = 0; i < modems_node.size(); ++i) {
        const YAML::Node& m = modems_node[i];
        if (!m["id"]) {
            std::ostringstream oss;
            oss << source << ": modem[" << i << "] missing required field 'id'";
            throw ScenarioLoadError(oss.str());
        }
        if (!m["usb_serial"]) {
            std::ostringstream oss;
            oss << source << ": modem[" << i << "] missing required field 'usb_serial'";
            throw ScenarioLoadError(oss.str());
        }

        ModemConfig mc;
        mc.id         = m["id"].as<std::string>();
        mc.usb_serial = m["usb_serial"].as<std::string>();

        if (modem_ids.count(mc.id)) {
            throw ScenarioLoadError(source + ": duplicate modem id '" + mc.id + "'");
        }
        modem_ids.insert(mc.id);

        if (!m["transducer_id"]) {
            throw ScenarioLoadError(source + ": modem '" + mc.id +
                "' missing required field 'transducer_id' "
                "(every modem must reference an entry in the top-level "
                "'transducers:' section)");
        }
        mc.transducer_id = m["transducer_id"].as<std::string>();
        if (!cfg.transducers.count(mc.transducer_id)) {
            throw ScenarioLoadError(source + ": modem '" + mc.id +
                "' transducer_id '" + mc.transducer_id +
                "' is not defined in the 'transducers:' section");
        }

        mc.clock_offset_ppm          = m["clock_offset_ppm"]          ? m["clock_offset_ppm"].as<float>()                : 0.0f;
        mc.velocity_radial_m_s       = m["velocity_radial_m_s"]       ? m["velocity_radial_m_s"].as<float>()             : 0.0f;
        mc.acceleration_radial_m_s2  = m["acceleration_radial_m_s2"]  ? m["acceleration_radial_m_s2"].as<float>()        : 0.0f;

        if (mc.clock_offset_ppm < 0.0f) {
            throw ScenarioLoadError(source + ": modem '" + mc.id +
                "' clock_offset_ppm must be >= 0 (it is a tolerance / σ, not a "
                "signed offset)");
        }

        cfg.modems.push_back(std::move(mc));
    }

    // Sample each modem's actual crystal offset from a truncated normal with
    // σ = its declared tolerance. Done after all modems are loaded so the
    // RNG iterates them in YAML order, giving reproducible draws under the
    // same random_seed regardless of channel layout.
    {
        std::mt19937_64 rng(cfg.random_seed);
        for (auto& mc : cfg.modems) {
            mc.actual_clock_offset_ppm =
                draw_truncated_normal_ppm(mc.clock_offset_ppm, rng);
        }
    }

    // --- channels ---
    if (!root["channels"]) {
        throw ScenarioLoadError(source + ": missing required section 'channels'");
    }
    const YAML::Node& channels_node = root["channels"];
    if (!channels_node.IsSequence() || channels_node.size() == 0) {
        throw ScenarioLoadError(source + ": 'channels' must be a non-empty sequence");
    }

    for (std::size_t i = 0; i < channels_node.size(); ++i) {
        const YAML::Node& c = channels_node[i];
        if (!c["from"]) {
            std::ostringstream oss;
            oss << source << ": channel[" << i << "] missing required field 'from'";
            throw ScenarioLoadError(oss.str());
        }
        if (!c["to"]) {
            std::ostringstream oss;
            oss << source << ": channel[" << i << "] missing required field 'to'";
            throw ScenarioLoadError(oss.str());
        }

        ChannelConfig cc;
        cc.from_modem = c["from"].as<std::string>();
        cc.to_modem   = c["to"].as<std::string>();

        // Validate modem references
        if (!modem_ids.count(cc.from_modem)) {
            throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                                    "] 'from' references unknown modem '" + cc.from_modem + "'");
        }
        if (!modem_ids.count(cc.to_modem)) {
            throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                                    "] 'to' references unknown modem '" + cc.to_modem + "'");
        }

        cc.range_m = c["range_m"] ? c["range_m"].as<float>() : 150.0f;
        if (cc.range_m <= 0.0f) {
            throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                                    "] 'range_m' must be > 0");
        }

        if (c["gain_db"]) cc.gain_db = c["gain_db"].as<float>();

        if (c["rx_atten_idx"]) {
            const int idx = c["rx_atten_idx"].as<int>();
            if (idx < 0 || idx > 1) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] 'rx_atten_idx' must be 0 or 1 (the modem reports two "
                    "input-attenuation pads)");
            }
            cc.rx_atten_idx = static_cast<uint8_t>(idx);
        }

        if (c["direct_los"]) cc.direct_los = c["direct_los"].as<bool>();

        if (c["propagation_delay_s"]) {
            cc.propagation_delay_s = c["propagation_delay_s"].as<float>();
            if (cc.propagation_delay_s < 0.0f) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                                        "] 'propagation_delay_s' must be >= 0");
            }
        }

        // --- mode + initial_range_m + geometry block (Session B) ---
        if (c["initial_range_m"]) {
            cc.initial_range_m = c["initial_range_m"].as<float>();
            if (cc.initial_range_m <= 0.0f) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] 'initial_range_m' must be > 0");
            }
        }

        if (c["mode"]) {
            const std::string mode_str = c["mode"].as<std::string>();
            if      (mode_str == "static")    cc.mode = ChannelMode::Static;
            else if (mode_str == "geometric") cc.mode = ChannelMode::Geometric;
            else {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] 'mode' must be 'static' or 'geometric' (got '" +
                    mode_str + "')");
            }
        }

        if (cc.mode == ChannelMode::Geometric) {
            if (!c["geometry"]) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] mode: geometric requires a 'geometry:' block");
            }
            if (c["multipath_taps"]) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] 'multipath_taps' is not allowed with mode: geometric — "
                    "taps are computed from the geometric scene each block");
            }
            const YAML::Node& g = c["geometry"];
            GeometricSceneConfig gs;
            if (g["water_depth_m"])         gs.water_depth_m         = g["water_depth_m"].as<float>();
            if (g["source_depth_m"])        gs.source_depth_m        = g["source_depth_m"].as<float>();
            if (g["receiver_depth_m"])      gs.receiver_depth_m      = g["receiver_depth_m"].as<float>();
            if (g["gamma_surface"])         gs.gamma_surface         = g["gamma_surface"].as<float>();
            if (g["gamma_bottom"])          gs.gamma_bottom          = g["gamma_bottom"].as<float>();
            if (g["spreading_exponent_k"])  gs.spreading_exponent_k  = g["spreading_exponent_k"].as<float>();
            if (g["enable_direct"])         gs.enable_direct         = g["enable_direct"].as<bool>();
            if (g["enable_surface"])        gs.enable_surface        = g["enable_surface"].as<bool>();
            if (g["enable_bottom"])         gs.enable_bottom         = g["enable_bottom"].as<bool>();
            if (g["enable_surface_bottom"]) gs.enable_surface_bottom = g["enable_surface_bottom"].as<bool>();
            if (g["enable_bottom_surface"]) gs.enable_bottom_surface = g["enable_bottom_surface"].as<bool>();
            if (g["r_min_m"])               gs.r_min_m               = g["r_min_m"].as<float>();
            if (g["r_max_m"])               gs.r_max_m               = g["r_max_m"].as<float>();

            if (gs.water_depth_m <= 0.0f) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] geometry.water_depth_m must be > 0");
            }
            if (gs.source_depth_m <= 0.0f || gs.source_depth_m >= gs.water_depth_m) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] geometry.source_depth_m must lie in (0, water_depth_m)");
            }
            if (gs.receiver_depth_m <= 0.0f || gs.receiver_depth_m >= gs.water_depth_m) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] geometry.receiver_depth_m must lie in (0, water_depth_m)");
            }
            if (gs.spreading_exponent_k <= 0.0f) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] geometry.spreading_exponent_k must be > 0");
            }

            // Default r_min / r_max bracket the R_0 used at message start.
            // R_0 = initial_range_m when provided, else range_m.
            const float r0 = (cc.initial_range_m > 0.0f)
                ? cc.initial_range_m : cc.range_m;
            if (gs.r_min_m <= 0.0f) gs.r_min_m = r0 * 0.5f;
            if (gs.r_max_m <= 0.0f) gs.r_max_m = r0 * 2.0f;
            if (!(gs.r_min_m < r0 && r0 < gs.r_max_m)) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                    "] geometry.r_min_m / r_max_m must bracket R_0 strictly "
                    "(r_min < R_0 < r_max)");
            }

            cc.geometry = gs;
        }

        // Inherit environment settings
        cc.spreading_factor = cfg.environment.spreading_factor;
        cc.saltwater        = cfg.environment.saltwater;
        cc.sound_speed_m_s  = cfg.environment.sound_speed_m_s;

        // Inherit source modem's velocity/acceleration; clock offset is the
        // *difference* between the two modems' sampled crystal offsets, so
        // loopback (src == rx) is always 0 and any crystal-tolerance setting
        // produces a non-zero relative offset between distinct modems.
        const ModemConfig* src = nullptr;
        const ModemConfig* rx  = nullptr;
        for (const auto& mc : cfg.modems) {
            if (mc.id == cc.from_modem) src = &mc;
            if (mc.id == cc.to_modem)   rx  = &mc;
        }
        if (src) {
            cc.velocity_radial_m_s      = src->velocity_radial_m_s;
            cc.acceleration_radial_m_s2 = src->acceleration_radial_m_s2;
        }
        if (src && rx) {
            cc.clock_offset_ppm = src->actual_clock_offset_ppm
                                - rx->actual_clock_offset_ppm;
        }

        // Multipath taps (optional, static mode only — geometric mode rejects
        // multipath_taps above and computes its taps from the scene).
        if (cc.mode == ChannelMode::Geometric) {
            // No-op: Channel populates taps_ from the scene at message start.
        } else if (const YAML::Node& taps_node = c["multipath_taps"]) {
            if (!taps_node.IsSequence()) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                                        "] 'multipath_taps' must be a sequence");
            }
            if (taps_node.size() > MAX_TAPS_PER_CHANNEL) {
                std::ostringstream oss;
                oss << source << ": channel[" << i << "] has " << taps_node.size()
                    << " taps, max is " << MAX_TAPS_PER_CHANNEL;
                throw ScenarioLoadError(oss.str());
            }

            bool has_zero_delay_tap = false;
            for (std::size_t t = 0; t < taps_node.size(); ++t) {
                const YAML::Node& tap = taps_node[t];
                MultipathTap mt;
                mt.delay_s     = tap["delay_s"]   ? tap["delay_s"].as<float>()   : 0.0f;
                const float gain_db = tap["gain_db"] ? tap["gain_db"].as<float>() : 0.0f;
                mt.gain_linear = std::pow(10.0f, gain_db / 20.0f);
                const float phase_deg = tap["phase_deg"] ? tap["phase_deg"].as<float>() : 0.0f;
                mt.phase_rad   = phase_deg * (3.14159265358979323846f / 180.0f);

                if (mt.delay_s < 0.0f || mt.delay_s > MAX_MULTIPATH_DELAY_S) {
                    std::ostringstream oss;
                    oss << source << ": channel[" << i << "] tap[" << t
                        << "] delay_s " << mt.delay_s << " outside [0, "
                        << MAX_MULTIPATH_DELAY_S << "]";
                    throw ScenarioLoadError(oss.str());
                }
                if (mt.delay_s == 0.0f) has_zero_delay_tap = true;
                cc.multipath_taps.push_back(mt);
            }

            if (!cc.direct_los && has_zero_delay_tap) {
                throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                                        "] has direct_los: false but a zero-delay "
                                        "tap was specified — when LOS is absent, "
                                        "the earliest tap should have delay_s > 0");
            }
        } else if (cc.direct_los) {
            // If no taps specified and LOS exists, add a single direct-path tap
            cc.multipath_taps.push_back({0.0f, 1.0f, 0.0f});
        } else {
            throw ScenarioLoadError(source + ": channel[" + std::to_string(i) +
                                    "] has direct_los: false and no multipath_taps; "
                                    "no rays would arrive at the receiver");
        }

        cfg.channels.push_back(std::move(cc));
    }

    // --- noise (optional) ---
    if (const YAML::Node& noise_node = root["noise"]) {
        cfg.noise.saltwater = cfg.environment.saltwater;  // inherit, may override
        if (noise_node["wenz_sea_state"])
            cfg.noise.wenz_sea_state = noise_node["wenz_sea_state"].as<int>();
        if (noise_node["level_above_noise_floor_db"]) {
            throw ScenarioLoadError(source +
                ": noise.level_above_noise_floor_db is no longer accepted — "
                "use noise.min_margin_above_afe_db (default 10 dB) to set the "
                "PSD margin above the modem's AFE noise floor, or "
                "noise.disable: true to short-circuit ambient noise.");
        }
        if (noise_node["min_margin_above_afe_db"])
            cfg.noise.min_margin_above_afe_db =
                noise_node["min_margin_above_afe_db"].as<float>();
        if (noise_node["disable"])
            cfg.noise.disable = noise_node["disable"].as<bool>();
        if (noise_node["saltwater"])
            cfg.noise.saltwater = noise_node["saltwater"].as<bool>();

        if (const YAML::Node& tonals_node = noise_node["tonal_sources"]) {
            if (tonals_node.IsSequence()) {
                for (const YAML::Node& t : tonals_node) {
                    dsp::TonalSource ts;
                    ts.frequency_hz      = t["frequency_hz"]      ? t["frequency_hz"].as<float>()      : 0.0f;
                    ts.amplitude_linear  = t["amplitude_linear"]  ? t["amplitude_linear"].as<float>()  : 0.0f;
                    ts.bandwidth_hz      = t["bandwidth_hz"]       ? t["bandwidth_hz"].as<float>()      : 0.0f;
                    cfg.noise.tonal_sources.push_back(ts);
                }
            }
        }
    }

    // --- logging (optional) ---
    if (const YAML::Node& log_node = root["logging"]) {
        if (log_node["log_raw_tx"])    cfg.logging.log_raw_tx    = log_node["log_raw_tx"].as<bool>();
        if (log_node["log_raw_rx"])    cfg.logging.log_raw_rx    = log_node["log_raw_rx"].as<bool>();
        if (log_node["log_processed"]) cfg.logging.log_processed = log_node["log_processed"].as<bool>();
        if (log_node["output_directory"]) cfg.logging.output_directory = log_node["output_directory"].as<std::string>();
        if (log_node["file_format"])   cfg.logging.file_format   = log_node["file_format"].as<std::string>();
        if (log_node["log_processing_time_histogram"])
            cfg.logging.log_processing_time_histogram = log_node["log_processing_time_histogram"].as<bool>();
        if (log_node["log_message_events"])
            cfg.logging.log_message_events            = log_node["log_message_events"].as<bool>();
        if (log_node["run_summary_path"])
            cfg.logging.run_summary_path              = log_node["run_summary_path"].as<std::string>();
    }

    return cfg;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

ScenarioConfig ScenarioLoader::load(const std::string& filepath) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filepath);
    } catch (const YAML::Exception& e) {
        throw ScenarioLoadError(filepath + ": YAML parse error: " + e.what());
    }
    return parse(root, filepath);
}

ScenarioConfig ScenarioLoader::load_from_string(const std::string& yaml_text) {
    YAML::Node root;
    try {
        root = YAML::Load(yaml_text);
    } catch (const YAML::Exception& e) {
        throw ScenarioLoadError(std::string("<string>: YAML parse error: ") + e.what());
    }
    return parse(root, "<string>");
}

} // namespace openCREST
