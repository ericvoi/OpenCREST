#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "config/scenario.hpp"
#include "simulator/metrics.hpp"
#include "simulator/processing_time_stats.hpp"

namespace openCREST {

// Per-run summary payload composed at shutdown. The simulator fills
// this in and hands it to write_run_summary_json which serializes to
// the location chosen by LoggingConfig.run_summary_path (or the
// default `<output_dir>/<scenario_name>_summary.json`).
//
// Fields are kept primitive so the writer is straight serialization
// with no policy. The schema matches the one documented in
// docs/paper_experiments/session_d_metrics_observability.md.
struct RunSummary {
    std::string                       scenario_name;
    std::string                       scenario_path;
    uint64_t                          random_seed = 0;
    std::string                       started_at;          // ISO-8601 UTC
    std::string                       ended_at;            // ISO-8601 UTC
    double                            duration_s = 0.0;
    std::vector<std::string>          modems;

    ProcessingTimeStats::Snapshot     processing_time{};

    // Mirrors the live Metrics struct, post-snapshot. We copy the
    // numbers we want to publish into POD so we don't have to drag the
    // atomic-bearing Metrics struct through the JSON layer.
    struct EngineCounters {
        uint64_t rx_ring_underruns = 0;
        uint64_t fw_rx_underruns   = 0;
        uint64_t tx_packets_total  = 0;
        uint64_t rx_packets_total  = 0;
    };
    EngineCounters                    channel_engine{};

    struct LogFiles {
        std::vector<std::string> events;
        std::vector<std::string> cdc;    // populated by the Python harness; empty here
        std::vector<std::string> wav;
    };
    LogFiles                          log_files{};
};

// Snapshot the live Metrics struct into RunSummary::EngineCounters.
// Pure helper — separated so it can be unit-tested independently.
RunSummary::EngineCounters snapshot_engine_counters(const Metrics& m);

// Compose the run summary's JSON form and write it to `path`. Returns
// false on filesystem error (caller may log and continue).
bool write_run_summary_json(const RunSummary& summary,
                            const std::string& path);

// Serialize to a string, primarily for tests and any in-memory consumer.
std::string serialize_run_summary(const RunSummary& summary);

// Resolve `LoggingConfig.run_summary_path` against the scenario for the
// case where the field is empty:
//   <output_directory>/<scenario_name>_summary.json
// If neither output_directory nor scenario_name is set, falls back to
// "run_summary.json" in the current working directory.
std::string resolve_run_summary_path(const ScenarioConfig& scenario);

} // namespace openCREST
