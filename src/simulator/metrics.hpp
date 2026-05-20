#pragma once
#include <atomic>
#include <cstdint>

namespace openCREST {

// Aggregate runtime counters, updated by I/O and processing threads.
// All fields are atomic for wait-free access from multiple threads.
// The main thread reads them periodically (once per second) for display.
struct Metrics {
    std::atomic<uint64_t> tx_packets_received{0};  // Upstream (modem → host)
    std::atomic<uint64_t> rx_packets_sent{0};       // Downstream (host → modem)
    std::atomic<uint64_t> sequence_gaps{0};         // TX packet sequence gaps
    std::atomic<uint64_t> short_reads{0};           // TX bulk reads < 512 bytes (ZLP/short)
    std::atomic<uint64_t> tx_ring_overruns{0};      // TX samples dropped (ring full)
    std::atomic<uint64_t> rx_ring_underruns{0};     // RX ring dry when engine ticks
    std::atomic<uint64_t> fw_rx_underruns{0};       // Status reports with modem RX underrun bit set
    std::atomic<uint64_t> fw_tx_overruns{0};        // Status reports with modem TX overrun bit set
    std::atomic<int32_t>  rx_drift{0};               // host tx_pkt_id − modem rx_expected_id (latest)
    std::atomic<uint64_t> processing_tick_us{0};    // Duration of last ChannelEngine tick

    // Clock-tracker telemetry (zero in PID mode).
    std::atomic<uint64_t> late_messages{0};         // Channel arrival was past ideal injection (gap clamped to 0)
    std::atomic<uint64_t> fill_anchor_fallbacks{0}; // Status referenced an unknown fill_reference_id
    std::atomic<int32_t>  estimated_fill_samples{0};// Latest extrapolated fill (most recent ModemIO update)
    std::atomic<uint32_t> estimated_modem_rate{0};  // Latest EWMA-smoothed Fs from clock tracker (Hz)
    std::atomic<uint32_t> fw_buffer_fill{0};        // Latest modem-reported buffer fill (samples)

    // Non-copyable (atomics)
    Metrics() = default;
    Metrics(const Metrics&)            = delete;
    Metrics& operator=(const Metrics&) = delete;

    void reset();

    // Print a one-line summary to stdout (called once per second).
    void print_summary(double elapsed_s) const;
};

} // namespace openCREST
