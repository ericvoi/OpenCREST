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

// Per-modem context passed into ChannelEngine. Carries everything needed
// to wire one modem into the PairBuffer / SourceWorker / ReceiverMix graph.
struct PerModemContext {
    std::string                id;
    CalibrationData            calibration;
    ModemRuntimeState*         runtime  = nullptr;
    SPSCRingBuffer<uint16_t>*  tx_ring  = nullptr;
    SPSCRingBuffer<uint16_t>*  rx_ring  = nullptr;
    dsp::NoiseConfig           noise_cfg;
    uint32_t                   sample_rate = 500'000;

    // Per-receiver dB boost applied to every Channel feeding this modem.
    // Set when the natural Wenz PSD at the receiver's fc sits below the
    // AFE noise floor + min_margin; preserves SNR at the cost of clipping
    // headroom.
    float                      receive_boost_db = 0.0f;
};

// Thin orchestrator owning per-pair PairBuffers, per-source workers, and
// per-receiver mixers.
//
// Lifecycle:
//   1. Construction builds Channel + PairBuffer for every (source,
//      receiver) pair, one SourceWorker per source, one ReceiverMix per
//      receiver.
//   2. start() spawns one thread per SourceWorker.
//   3. ModemIO threads pull receiver-side via receiver_mix(idx).
//   4. stop() / destructor joins all worker threads.
//
// PairBuffer sizing covers the worst-case write extent across all channels,
// with an in-flight slack of environment.max_message_duration_s *
// sample_rate. The in-flight slack matters because half-duplex receivers
// cannot drain while their modem is in TX — the entire message must fit.
//
// The engine holds no propagation-model knowledge. Channels are constructed
// first and each reports its own write extent (base delay + longest tap)
// through Channel::write_extent_samples(); the engine takes the worst and
// sizes every PairBuffer from it.
class ChannelEngine {
public:
    ChannelEngine(const ScenarioConfig&        scenario,
                  std::vector<PerModemContext> modems);
    ~ChannelEngine();

    ChannelEngine(const ChannelEngine&)            = delete;
    ChannelEngine& operator=(const ChannelEngine&) = delete;

    // Spawn one thread per SourceWorker
    void start();

    // Stop and join all worker threads. Called by the destructor.
    void stop();

    // ReceiverMix for the given modem; nullptr if out of range. Called
    // from ModemIO::send_rx_data on the receiver clock.
    ReceiverMix* receiver_mix(size_t modem_idx);

    // Wire ModemIO-owned per-modem trackers into the SourceWorker graph.
    // `fill_tracker` and `rx_ring` belong to `modem_idx` and are queried
    // by source workers whose channels feed into this modem;
    // `tx_start_estimator` belongs to `modem_idx` and is queried by the
    // source worker driving this modem's own outgoing channels.
    // Used by the clock-tracker arrival-alignment path; recorded but
    // ignored in PID mode.
    void wire_modem_trackers(size_t modem_idx,
                             IFillTracker* fill_tracker,
                             TxStartEstimator* tx_start_estimator,
                             SPSCRingBuffer<uint16_t>* rx_ring);

    // Install the process-wide Metrics into every SourceWorker. Safe to
    // call between construction and start().
    void set_metrics(Metrics* metrics);

    // Install the shared processing-time histogram. `deadline_us` flags
    // a per-batch tick as a real-time underrun (0 disables).
    void set_processing_time_stats(ProcessingTimeStats* stats,
                                    uint64_t deadline_us);

    // Install per-source-modem event logs. `logs.size()` must equal
    // num_modems(); null entries leave that source uninstrumented.
    void set_message_event_logs(const std::vector<MessageEventLog*>& logs);

    // SourceWorker for a given source modem; nullptr if that modem has no
    // outgoing channels (no worker is created). Used by tests that drive
    // workers directly.
    SourceWorker* source_worker(size_t modem_idx);

    size_t num_modems()        const { return modems_.size(); }
    size_t num_pair_buffers()  const { return pair_buffers_.size(); }
    size_t num_source_workers() const { return source_workers_.size(); }

    // PairBuffer capacity in slots (post power-of-2 rounding). All pair
    // buffers share the same capacity; 0 if none exist.
    size_t pair_buffer_capacity_samples() const {
        return pair_buffers_.empty() ? 0 : pair_buffers_.front()->capacity();
    }

    // PairBuffer capacity (pre power-of-2 rounding) for a given worst-case
    // per-channel write extent, which the engine takes from the constructed
    // Channels via Channel::write_extent_samples(). Adds the environment-level
    // range floor and the in-flight message slack.
    //
    // Model-agnostic by construction: `worst_channel_extent` is the only
    // channel-derived input, and every propagation model reports it through
    // the same accessor. Public for tests and budget inspection.
    static size_t pair_capacity_for_extent(size_t                worst_channel_extent,
                                           const ScenarioConfig& scenario,
                                           uint32_t              sample_rate);

private:
    size_t modem_index(const std::string& id) const;

    // Per-receiver list of (source_idx, outgoing_idx) entries feeding
    // into that receiver. Used by wire_modem_trackers.
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
