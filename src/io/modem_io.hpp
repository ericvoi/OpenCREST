#pragma once
#include "modem/modem.hpp"
#include "core/ring_buffer.hpp"
#include "core/constants.hpp"
#include "protocol/protocol_codec.hpp"
#include "simulator/metrics.hpp"
#include "core/fill_tracker.hpp"
#include "core/tx_start_estimator.hpp"
#include "logging/stream_logger.hpp"
#include "channel/receiver_mix.hpp"
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <memory>

namespace openCREST {

// Per-modem I/O thread.
//
// Bridges between the modem's USB interface and the SPSC ring buffers shared
// with the ChannelEngine. One ModemIO instance per modem.
//
// Polling loop (see architecture.md §9):
//   Every CONTROL_POLL_INTERVAL iterations: poll_control() → update runtime state
//   If state == TX:  recv_data → push to tx_ring
//   If state == RX:  pull from rx_ring, send_data (if pacer allows)
//   If state == SETTLING or IDLE: no data transfer
//
// The thread is started externally (caller creates std::thread calling run()).
// Call stop() from any thread to request clean exit; the loop exits after the
// current iteration.
class ModemIO {
public:
    // `receiver_mix`, `logger`, and `metrics` are optional; may be nullptr.
    // `receiver_mix` is used by send_rx_data() to top up rx_ring on demand
    // (one packet's worth at a time); when nullptr, ModemIO drains rx_ring
    // as filled by an external producer (legacy / test mode).
    ModemIO(Modem&                           modem,
            SPSCRingBuffer<uint16_t>&        tx_ring,
            SPSCRingBuffer<uint16_t>&        rx_ring,
            ReceiverMix*                     receiver_mix = nullptr,
            logging::StreamLogger*           logger       = nullptr,
            Metrics*                         metrics      = nullptr);

    // Thread entry point. Blocks until stop() is called.
    void run();

    // Signal the polling loop to exit. Thread-safe.
    void stop();

    bool is_running() const {
        return running_.load(std::memory_order_relaxed);
    }

    // Expose gap count for metrics
    uint64_t sequence_gap_count() const { return codec_.gap_count(); }

    // Accessors for ChannelEngine wiring (Phase 4 arrival alignment).
    // The tracker tracks this modem's RX-buffer fill (host→modem
    // direction); the estimator tracks when this modem started its
    // current TX burst (modem→host direction). Both are queried by
    // SourceWorker / ChannelEngine across modem pairs.
    IFillTracker&     fill_tracker()        { return *tracker_; }
    TxStartEstimator& tx_start_estimator()  { return tx_start_estimator_; }

    // Print one diagnostic line for this modem's tracker state. Called
    // from the Simulator's per-second metrics loop. Reads from atomic
    // snapshots updated in handle_status / send_rx_data, so this is
    // safe to invoke from a different thread. `elapsed_s` is the
    // wall-clock interval since the previous print, used to compute
    // per-modem RX pkt/s.
    void print_metrics_line(double elapsed_s);

private:
    void poll_control();
    void handle_status(const protocol::StatusPayload& status);
    void receive_tx_data();
    void send_rx_data();

    // Decode/sequence-check/forward a single TX packet sitting in data_buf_
    // after a successful recv_data(). Shared between the steady-state TX
    // path and the on-exit drain so trailing packets feed the channel
    // pipeline identically to in-session ones.
    void process_tx_packet(int bytes_transferred);

    // Pull any TX packets still parked in the modem's USB FIFO into the
    // host before letting the state transition complete. The firmware's
    // resetHil() does not actually discard pending TX data (TinyUSB's
    // tud_vendor_n_write_flush stages rather than drops), so without this
    // the last packet of one TX session would surface at the head of the
    // next one and trip sequence-gap detection.
    void drain_tx_pipeline();

    Modem&                    modem_;
    SPSCRingBuffer<uint16_t>& tx_ring_;
    SPSCRingBuffer<uint16_t>& rx_ring_;
    ReceiverMix*              receiver_mix_;
    logging::StreamLogger*    logger_;
    Metrics*                  metrics_;

    protocol::ProtocolCodec        codec_;
    CalibrationData                cal_;       // Captured at construction (before tracker_)
    std::unique_ptr<IFillTracker>  tracker_;
    TxStartEstimator               tx_start_estimator_;
    size_t                         samples_per_pkt_;

    std::atomic<bool>  running_{false};
    uint16_t           tx_pkt_id_ = 0;    // Sequence ID for host→modem RX stream
    uint32_t           last_error_flags_ = 0;  // previous value, for transition logging
    bool               log_first_rx_send_ = true;  // one-shot tx_pkt_id_ log per RX entry
    // First status after an RX entry shows pipeline-fill drift vs. the pre-reset
    // prev value — a false positive. Skip the drop check on the very first
    // status in each RX session; just seed the baseline.
    bool               skip_next_drift_check_ = true;
    // Tracker's fallback-anchor count, observed last status. Used to
    // compute the delta into the process-wide metric.
    uint64_t           last_fallback_count_ = 0;

    // Per-modem snapshots for the diagnostic line. Updated on each
    // status (handle_status) / send-skip (send_rx_data); read by the
    // Simulator main thread via print_metrics_line().
    std::atomic<uint16_t> last_fill_fw_{0};            // Modem-reported buffer_fill
    std::atomic<int32_t>  last_fill_est_{0};           // Tracker's extrapolated fill at last status
    std::atomic<uint32_t> last_modem_rate_{0};         // Tracker's EWMA-smoothed Fs estimate
    std::atomic<uint64_t> last_tracker_fallbacks_{0};  // Cumulative fallback_anchor_count
    std::atomic<uint64_t> rx_send_skipped_{0};         // send_rx_data early-returns due to empty rx_ring
    std::atomic<uint64_t> rx_packets_sent_{0};         // Cumulative packets sent host→modem (this modem only)
    uint64_t              prev_rx_packets_sent_ = 0;   // Snapshot at last print, for per-modem pkt/s

    // Pre-allocated scratch buffers (stack-allocated; size known at compile time)
    alignas(64) uint8_t  data_buf_[DATA_PACKET_SIZE];
    alignas(64) uint8_t  ctrl_buf_[CONTROL_PACKET_SIZE];

    static constexpr int  CTRL_TIMEOUT_MS          = 1;
    static constexpr int  DATA_TIMEOUT_MS          = 1;
    // Poll every 5 sends. Increasing this to reduce libusb-timeout overhead
    // starved the pacer of fresh fill measurements and widened the window
    // during which buffer state could drift between updates. Keep it tight.
    static constexpr size_t CONTROL_POLL_INTERVAL  = 5;

    // Bound on the on-exit TX-FIFO drain. The modem's TinyUSB TX FIFO holds
    // a small handful of packets at most; 8 leaves headroom while keeping
    // the worst-case wall time bounded (8 × TX_DRAIN_TIMEOUT_MS = 8 ms).
    static constexpr int  TX_DRAIN_MAX_ITERATIONS  = 8;
    static constexpr int  TX_DRAIN_TIMEOUT_MS      = 1;
};

} // namespace openCREST
