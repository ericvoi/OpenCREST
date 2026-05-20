#include "simulator/simulator.hpp"
#include <spdlog/spdlog.h>
#include <csignal>
#include <cstdlib>
#include <atomic>
#include <string>

namespace {

// Global pointer so the signal handler can call stop()
openCREST::Simulator* g_simulator = nullptr;
std::atomic<bool>     g_stop_requested{false};

void signal_handler(int /*sig*/) {
    g_stop_requested.store(true, std::memory_order_relaxed);
    if (g_simulator) g_simulator->stop();
}

void print_usage(const char* argv0) {
    std::printf("Usage: %s <scenario.yaml>\n", argv0);
    std::printf("  scenario.yaml  Path to a scenario YAML file\n");
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

    if (argc < 2) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string scenario_path = argv[1];

    // Install signal handlers for clean shutdown
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    openCREST::Simulator sim(scenario_path);
    g_simulator = &sim;

    if (!sim.initialize()) {
        spdlog::error("Initialization failed. Aborting.");
        return EXIT_FAILURE;
    }

    sim.run();

    return EXIT_SUCCESS;
}
