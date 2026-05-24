#pragma once
#include "config/scenario.hpp"
#include "config/scenario_loader.hpp"
#include "modem/modem.hpp"
#include "modem/modem_registry.hpp"
#include "channel/channel_engine.hpp"
#include "core/ring_buffer.hpp"
#include "core/constants.hpp"
#include "io/modem_io.hpp"
#include "logging/stream_logger.hpp"
#include "simulator/metrics.hpp"
#include "simulator/processing_time_stats.hpp"
#include "simulator/message_event_log.hpp"
#include "simulator/run_summary.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace openCREST {

// Top-level coordinator: loads scenario, discovers modems, runs the
// simulation loop, and shuts down cleanly on request.
//
// Lifecycle:
//   Simulator sim(scenario_path);
//   if (!sim.initialize()) { /* error */ }
//   sim.run();   // blocks until stop() or fatal error
class Simulator {
public:
    explicit Simulator(std::string scenario_path);
    ~Simulator();

    Simulator(const Simulator&)            = delete;
    Simulator& operator=(const Simulator&) = delete;

    // Load scenario, connect modems, calibrate, build engine.
    // Returns false on any fatal error (message already logged).
    bool initialize();

    // Enter HIL mode, start threads, run the metrics loop. Blocks until
    // stop() or any I/O thread signals a fatal error.
    void run();

    // Signal a clean shutdown from any thread.
    void stop();

    const Metrics& metrics() const { return metrics_; }

private:
    bool load_scenario();
    bool discover_modems();
    bool calibrate_modems();
    bool build_channel_engine();

    void start_io_threads();
    void join_all_threads();

    std::string       scenario_path_;
    ScenarioConfig    scenario_{};

    ModemRegistry     registry_;
    std::vector<std::unique_ptr<Modem>> modems_;

    // One pair per modem; owned here, referenced by IO + engine.
    struct ModemBuffers {
        SPSCRingBuffer<uint16_t> tx_ring{TX_RING_CAPACITY};
        SPSCRingBuffer<uint16_t> rx_ring{RX_RING_CAPACITY};
    };
    std::vector<ModemBuffers> buffers_;

    std::unique_ptr<ChannelEngine> engine_;
    std::vector<std::unique_ptr<ModemIO>> io_workers_;
    std::unique_ptr<logging::StreamLogger> logger_;
    Metrics metrics_;

    // Observability sinks; pointers handed to SourceWorkers via ChannelEngine.
    std::unique_ptr<ProcessingTimeStats>          processing_time_stats_;
    std::vector<std::unique_ptr<MessageEventLog>> message_event_logs_;
    std::chrono::system_clock::time_point         run_started_at_{};
    std::chrono::steady_clock::time_point         run_started_steady_{};

    void install_observability();   // open log files, wire engine pointers
    void emit_run_summary();        // gather snapshot, write JSON

    // SourceWorker threads live inside ChannelEngine.
    std::vector<std::thread> io_threads_;

    std::atomic<bool> running_{false};
};

} // namespace openCREST
