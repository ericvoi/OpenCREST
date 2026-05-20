#include "simulator/run_summary.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

namespace openCREST {

using nlohmann::json;

RunSummary::EngineCounters snapshot_engine_counters(const Metrics& m) {
    RunSummary::EngineCounters c{};
    c.rx_ring_underruns = m.rx_ring_underruns.load(std::memory_order_relaxed);
    c.fw_rx_underruns   = m.fw_rx_underruns.load  (std::memory_order_relaxed);
    c.tx_packets_total  = m.tx_packets_received.load(std::memory_order_relaxed);
    c.rx_packets_total  = m.rx_packets_sent.load   (std::memory_order_relaxed);
    return c;
}

std::string resolve_run_summary_path(const ScenarioConfig& scenario) {
    if (!scenario.logging.run_summary_path.empty()) {
        return scenario.logging.run_summary_path;
    }
    const std::string& dir  = scenario.logging.output_directory;
    const std::string& name = scenario.name;
    std::string base = name.empty() ? std::string{"run"} : name;
    if (dir.empty()) return base + "_summary.json";
    if (dir.back() == '/') return dir + base + "_summary.json";
    return dir + "/" + base + "_summary.json";
}

std::string serialize_run_summary(const RunSummary& s) {
    json j;
    j["scenario_name"] = s.scenario_name;
    j["scenario_path"] = s.scenario_path;
    j["random_seed"]   = s.random_seed;
    j["started_at"]    = s.started_at;
    j["ended_at"]      = s.ended_at;
    j["duration_s"]    = s.duration_s;
    j["modems"]        = s.modems;

    j["processing_time"] = {
        {"count",          s.processing_time.count},
        {"mean_us",        s.processing_time.mean_us},
        {"p50_us",         s.processing_time.p50_us},
        {"p95_us",         s.processing_time.p95_us},
        {"p99_us",         s.processing_time.p99_us},
        {"max_us",         s.processing_time.max_us},
        {"underrun_count", s.processing_time.underrun_count},
    };

    j["channel_engine"] = {
        {"rx_ring_underruns", s.channel_engine.rx_ring_underruns},
        {"fw_rx_underruns",   s.channel_engine.fw_rx_underruns},
        {"tx_packets_total",  s.channel_engine.tx_packets_total},
        {"rx_packets_total",  s.channel_engine.rx_packets_total},
    };

    j["log_files"] = {
        {"events", s.log_files.events},
        {"cdc",    s.log_files.cdc},
        {"wav",    s.log_files.wav},
    };
    return j.dump(2);
}

bool write_run_summary_json(const RunSummary& summary,
                            const std::string& path) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) return false;
    out << serialize_run_summary(summary);
    return static_cast<bool>(out);
}

} // namespace openCREST
