#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "config/scenario.hpp"
#include "simulator/metrics.hpp"
#include "simulator/processing_time_stats.hpp"

namespace openCREST {

// Per-run summary payload composed at shutdown. The simulator fills this
// in and hands it to write_run_summary_json, which serializes to the
// location chosen by LoggingConfig.run_summary_path (default
// `<output_dir>/<scenario_name>_summary.json`).
struct RunSummary {
    std::string                       scenario_name;
    std::string                       scenario_path;
    uint64_t                          random_seed = 0;
    std::string                       started_at;          // ISO-8601 UTC
    std::string                       ended_at;            // ISO-8601 UTC
    double                            duration_s = 0.0;
    std::vector<std::string>          modems;

    ProcessingTimeStats::Snapshot     processing_time{};

    // Snapshot of selected Metrics fields as POD so the JSON layer doesn't
    // need to touch the atomic-bearing struct.
    struct EngineCounters {
        uint64_t rx_ring_underruns = 0;
        uint64_t fw_rx_underruns   = 0;
        uint64_t tx_packets_total  = 0;
        uint64_t rx_packets_total  = 0;
    };
    EngineCounters                    channel_engine{};

    struct LogFiles {
        std::vector<std::string> events;
        std::vector<std::string> cdc;    // populated by the Python harness
        std::vector<std::string> wav;
    };
    LogFiles                          log_files{};
};

// Snapshot live Metrics into RunSummary::EngineCounters.
RunSummary::EngineCounters snapshot_engine_counters(const Metrics& m);

// Write the summary as JSON. Returns false on filesystem error.
bool write_run_summary_json(const RunSummary& summary,
                            const std::string& path);

// Serialize to a string (tests, in-memory consumers).
std::string serialize_run_summary(const RunSummary& summary);

// Resolve LoggingConfig.run_summary_path against the scenario when empty:
//   <output_directory>/<scenario_name>_summary.json
// With neither set, falls back to "run_summary.json" in the CWD.
std::string resolve_run_summary_path(const ScenarioConfig& scenario);

} // namespace openCREST
