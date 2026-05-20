#include "simulator/metrics.hpp"
#include <cstdio>

namespace openCREST {

void Metrics::reset() {
    tx_packets_received.store(0, std::memory_order_relaxed);
    rx_packets_sent.store(0,     std::memory_order_relaxed);
    sequence_gaps.store(0,       std::memory_order_relaxed);
    short_reads.store(0,         std::memory_order_relaxed);
    tx_ring_overruns.store(0,    std::memory_order_relaxed);
    rx_ring_underruns.store(0,   std::memory_order_relaxed);
    fw_rx_underruns.store(0,     std::memory_order_relaxed);
    fw_tx_overruns.store(0,      std::memory_order_relaxed);
    // rx_drift is a level, not a counter — do not reset on interval rollover
    processing_tick_us.store(0,  std::memory_order_relaxed);
    late_messages.store(0,        std::memory_order_relaxed);
    fill_anchor_fallbacks.store(0, std::memory_order_relaxed);
    // estimated_fill_samples, estimated_modem_rate, fw_buffer_fill are
    // levels — do not reset on interval rollover
}

void Metrics::print_summary(double elapsed_s) const {
    const double tx_pps = elapsed_s > 0.0
        ? static_cast<double>(tx_packets_received.load(std::memory_order_relaxed)) / elapsed_s
        : 0.0;
    const double rx_pps = elapsed_s > 0.0
        ? static_cast<double>(rx_packets_sent.load(std::memory_order_relaxed)) / elapsed_s
        : 0.0;

    std::printf("[metrics] TX %.0f pkt/s  RX %.0f pkt/s  "
                "gaps %llu  short %llu  tx_ovr %llu  rx_udr %llu  "
                "fw_udr %llu  fw_ovr %llu  drift %+d  tick %llu us  "
                "late %llu  fb %llu  fill_est %d  fill_fw %u  rate %u\n",
                tx_pps, rx_pps,
                static_cast<unsigned long long>(sequence_gaps.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(short_reads.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(tx_ring_overruns.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(rx_ring_underruns.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(fw_rx_underruns.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(fw_tx_overruns.load(std::memory_order_relaxed)),
                rx_drift.load(std::memory_order_relaxed),
                static_cast<unsigned long long>(processing_tick_us.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(late_messages.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(fill_anchor_fallbacks.load(std::memory_order_relaxed)),
                estimated_fill_samples.load(std::memory_order_relaxed),
                fw_buffer_fill.load(std::memory_order_relaxed),
                estimated_modem_rate.load(std::memory_order_relaxed));
    std::fflush(stdout);
}

} // namespace openCREST
