#pragma once
#include <atomic>
#include <cstdint>

namespace openCREST {

// Aggregate runtime counters updated by I/O and processing threads.
// All fields are atomic for wait-free multi-thread access; the main thread
// reads them once per second for display.
struct Metrics {
    std::atomic<uint64_t> tx_packets_received{0};  // modem → host
    std::atomic<uint64_t> rx_packets_sent{0};       // host → modem
    std::atomic<uint64_t> sequence_gaps{0};         // TX packet sequence gaps
    std::atomic<uint64_t> short_reads{0};           // TX bulk reads < 512 bytes (ZLP/short)
    std::atomic<uint64_t> tx_ring_overruns{0};      // TX samples dropped (ring full)
    std::atomic<uint64_t> rx_ring_underruns{0};     // RX ring dry when engine ticks
    std::atomic<uint64_t> fw_rx_underruns{0};       // status reports modem RX underrun
    std::atomic<uint64_t> fw_tx_overruns{0};        // status reports modem TX overrun
    std::atomic<int32_t>  rx_drift{0};               // host tx_pkt_id − modem rx_expected_id
    std::atomic<uint64_t> processing_tick_us{0};    // duration of last ChannelEngine tick

    // Clock-tracker telemetry.
    std::atomic<uint64_t> late_messages{0};         // arrival past ideal injection (gap clamped to 0)
    std::atomic<uint64_t> fill_anchor_fallbacks{0}; // status referenced an unknown fill_reference_id
    std::atomic<int32_t>  estimated_fill_samples{0};// extrapolated fill from most recent ModemIO update
    std::atomic<uint32_t> estimated_modem_rate{0};  // EWMA-smoothed modem Fs estimate (Hz)
    std::atomic<uint32_t> fw_buffer_fill{0};        // modem-reported buffer fill (samples)

    Metrics() = default;
    Metrics(const Metrics&)            = delete;
    Metrics& operator=(const Metrics&) = delete;

    void reset();

    // One-line stdout summary; called once per second.
    void print_summary(double elapsed_s) const;
};

} // namespace openCREST
