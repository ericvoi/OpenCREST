#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "config/scenario.hpp"
#include "core/types.hpp"
#include "core/ring_buffer.hpp"
#include "dsp/noise_generator.hpp"
#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "channel/receiver_mix.hpp"
#include "channel/source_worker.hpp"

namespace openCREST {

// Per-modem context passed into ChannelEngine.
//
// Provides everything the engine needs to wire one modem into the
// PairBuffer / SourceWorker / ReceiverMix graph: identification, the
// shared atomic runtime state, the pair of ring buffers, and the
// ambient-noise configuration (turned into a NoiseGenerator inside the
// ReceiverMix at construction).
struct PerModemContext {
    std::string                id;
    CalibrationData            calibration;
    ModemRuntimeState*         runtime  = nullptr;
    SPSCRingBuffer<uint16_t>*  tx_ring  = nullptr;
    SPSCRingBuffer<uint16_t>*  rx_ring  = nullptr;
    dsp::NoiseConfig           noise_cfg;
    uint32_t                   sample_rate = 500'000;

    // Per-receiver dB boost applied to every Channel feeding this modem.
    // Computed at scenario load when the natural Wenz PSD at the modem's
    // center frequency would otherwise sit below the modem's AFE noise
    // floor + min_margin. Preserves SNR; eats clipping headroom.
    float                      receive_boost_db = 0.0f;
};

// Phase 2 ChannelEngine: a thin orchestrator that owns the per-pair
// buffers, the per-source workers, and the per-receiver mixers.
//
// Lifetime
// --------
// 1. Construction: parses scenario.channels, builds Channel + PairBuffer
//    for every (source, receiver) pair, builds one SourceWorker per source
//    modem (collecting all of its outgoing channels), and builds one
//    ReceiverMix per receiver (collecting all incoming PairBuffers).
// 2. start() — spawns one std::thread per SourceWorker.
// 3. (simulation runs; ModemIO threads call receiver_mix(idx)->pull(...))
// 4. stop() / destructor — joins all worker threads.
//
// Sizing
// ------
// Every PairBuffer is sized for the worst case derived from
//   environment.max_range_m       (falls back to largest channel range)
// plus MAX_MULTIPATH_DELAY_S worth of taps
// plus environment.max_message_duration_s × sample_rate of in-flight slack.
// The in-flight term is required because the receiver only drains while its
// modem is in RX state — in half-duplex loopback the entire message has to
// fit in the buffer before any of it can be consumed.
class ChannelEngine {
public:
    ChannelEngine(const ScenarioConfig&        scenario,
                  std::vector<PerModemContext> modems);
    ~ChannelEngine();

    ChannelEngine(const ChannelEngine&)            = delete;
    ChannelEngine& operator=(const ChannelEngine&) = delete;

    // Spawn one thread per SourceWorker. Idempotent.
    void start();

    // Stop and join all worker threads. Idempotent. Safe to call from any
    // thread. Called automatically by the destructor.
    void stop();

    // Get the receiver mix for the modem at `modem_idx`. Used by ModemIO's
    // send_rx_data to pull samples on the receiver clock. Returns nullptr
    // if the index is out of range.
    ReceiverMix* receiver_mix(size_t modem_idx);

    // Wire ModemIO-owned per-modem trackers into the engine's
    // SourceWorker graph. Called by Simulator AFTER each modem's
    // ModemIO has been constructed. In clock-tracker mode the
    // SourceWorker uses these to compute precise message-arrival
    // alignment; in PID mode they're recorded but ignored.
    //
    // `fill_tracker` and `rx_ring` belong to modem `modem_idx` and
    // are queried by source workers whose channels feed *into* this
    // modem. `tx_start_estimator` belongs to modem `modem_idx` and is
    // queried by the source worker driving this modem's own outgoing
    // channels.
    void wire_modem_trackers(size_t modem_idx,
                             IFillTracker* fill_tracker,
                             TxStartEstimator* tx_start_estimator,
                             SPSCRingBuffer<uint16_t>* rx_ring);

    // Forwarding setter: hands the process-wide Metrics struct to
    // every SourceWorker for incrementing arrival-alignment counters
    // (late_messages). Idempotent; safe to call after construction
    // and before start().
    void set_metrics(Metrics* metrics);

    // Session D — install the shared processing-time histogram into
    // every SourceWorker. `deadline_us` is used to flag a per-batch
    // tick as a real-time underrun (0 disables that path). Idempotent;
    // safe to call before start().
    void set_processing_time_stats(ProcessingTimeStats* stats,
                                    uint64_t deadline_us);

    // Session D — install the per-source-modem message event log.
    // `logs.size()` must equal num_modems(); any null entry leaves
    // that source uninstrumented.
    void set_message_event_logs(const std::vector<MessageEventLog*>& logs);

    // Look up a SourceWorker by source-modem index; returns nullptr if
    // the index has no outgoing channels (and therefore no worker).
    // Used by the simulator and integration tests to drive workers
    // directly (e.g., for tests that bypass start()/stop()).
    SourceWorker* source_worker(size_t modem_idx);

    size_t num_modems()        const { return modems_.size(); }
    size_t num_pair_buffers()  const { return pair_buffers_.size(); }
    size_t num_source_workers() const { return source_workers_.size(); }

    // Capacity (slots, post power-of-2 rounding) of the engine's PairBuffers.
    // All pair buffers share the same capacity. Returns 0 if no pair buffers.
    size_t pair_buffer_capacity_samples() const {
        return pair_buffers_.empty() ? 0 : pair_buffers_.front()->capacity();
    }

    // Compute worst-case PairBuffer capacity from scenario (pre power-of-2
    // rounding). Public so tests and tools can inspect the budget without
    // constructing the engine.
    static size_t worst_case_pair_capacity(const ScenarioConfig& scenario,
                                            uint32_t sample_rate);

private:
    // Resolve modem id → index.
    size_t modem_index(const std::string& id) const;

    // For wire_modem_trackers: per receiver_modem_idx, the list of
    // (source_modem_idx, outgoing_idx_in_source_worker) entries that
    // feed into that receiver. Built at construction; consumed only
    // when wiring is requested.
    struct OutgoingRef { size_t source_idx; size_t outgoing_idx; };

    std::vector<PerModemContext>                   modems_;
    std::vector<std::unique_ptr<PairBuffer>>       pair_buffers_;
    std::vector<std::unique_ptr<SourceWorker>>     source_workers_;
    std::vector<std::unique_ptr<ReceiverMix>>      receiver_mixes_;
    std::vector<std::thread>                       worker_threads_;
    std::vector<std::vector<OutgoingRef>>          per_receiver_outgoing_refs_;

    std::atomic<bool> started_{false};
};

} // namespace openCREST
