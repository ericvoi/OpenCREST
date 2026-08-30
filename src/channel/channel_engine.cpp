#include "channel/channel_engine.hpp"
#include "core/constants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace openCREST {

namespace {

double clamp_sound_speed_engine(float speed) {
    return (speed > 0.0f) ? static_cast<double>(speed) : 1500.0;
}

} // namespace

size_t ChannelEngine::modem_index(const std::string& id) const {
    for (size_t i = 0; i < modems_.size(); ++i) {
        if (modems_[i].id == id) return i;
    }
    throw std::invalid_argument("ChannelEngine: unknown modem id '" + id + "'");
}

size_t ChannelEngine::pair_capacity_for_extent(size_t                worst_channel_extent,
                                               const ScenarioConfig& scenario,
                                               uint32_t              sample_rate) {
    const double sound_speed = clamp_sound_speed_engine(
        scenario.environment.sound_speed_m_s);

    // Environment-level floor: a scenario may declare a worst-case range that
    // exceeds any individual channel's configured extent (e.g. modems that
    // move further apart than range_m suggests). Range-derived, plus the
    // multipath allowance the scenario loader caps tap delays at.
    if (scenario.environment.max_range_m > 0.0f) {
        const size_t env_base = static_cast<size_t>(std::round(
            static_cast<double>(scenario.environment.max_range_m) *
            sample_rate / sound_speed));
        const size_t env_mp = static_cast<size_t>(std::round(
            MAX_MULTIPATH_DELAY_S * sample_rate));
        worst_channel_extent = std::max(worst_channel_extent, env_base + env_mp);
    }
    if (worst_channel_extent == 0) worst_channel_extent = sample_rate;  // floor

    // In-flight slack: hold an entire max-duration message because the
    // receiver does not drain while its modem is in TX (half-duplex) or
    // while its pull-thread is otherwise idle.
    float msg_dur_s = scenario.environment.max_message_duration_s;
    if (!(msg_dur_s > 0.0f)) msg_dur_s = 10.0f;
    const size_t in_flight = static_cast<size_t>(
        std::round(static_cast<double>(msg_dur_s) * sample_rate));

    return worst_channel_extent + in_flight;
}

ChannelEngine::ChannelEngine(const ScenarioConfig&         scenario,
                             std::vector<PerModemContext>  modems)
    : modems_(std::move(modems))
{
    if (modems_.empty()) {
        throw std::invalid_argument("ChannelEngine: at least one modem required");
    }

    const uint32_t sample_rate = modems_[0].sample_rate;

    // Resolve each modem's TransducerSpec. Loader guarantees every
    // transducer_id resolves
    std::vector<TransducerSpec> transducer_per_modem(modems_.size());
    for (size_t i = 0; i < modems_.size(); ++i) {
        for (const auto& mc : scenario.modems) {
            if (mc.id != modems_[i].id) continue;
            const auto it = scenario.transducers.find(mc.transducer_id);
            if (it != scenario.transducers.end()) {
                transducer_per_modem[i] = it->second;
            }
            break;
        }
    }

    // Build per-source outgoing lists and per-receiver incoming lists.
    std::vector<std::vector<SourceWorker::Outgoing>> per_source_outgoing(
        modems_.size());
    std::vector<std::vector<PairBuffer*>> per_receiver_incoming(
        modems_.size());
    per_receiver_outgoing_refs_.assign(modems_.size(), {});

    pair_buffers_.reserve(scenario.channels.size());

    // Pass 1: build every Channel and let each report its own write extent.
    // Whichever propagation model a channel uses, the extent arrives through
    // the same accessor — the engine never inspects cc.mode.
    struct BuiltChannel {
        std::unique_ptr<Channel> channel;
        size_t                   src_idx;
        size_t                   rcv_idx;
    };
    std::vector<BuiltChannel> built;
    built.reserve(scenario.channels.size());

    size_t worst_channel_extent = 0;
    for (const auto& cc : scenario.channels) {
        const size_t src_idx = modem_index(cc.from_modem);
        const size_t rcv_idx = modem_index(cc.to_modem);

        // The receiver's boost (not the source's) applies. Boost preserves
        // SNR at the receiver, so signal and noise must scale together
        // equally.
        auto channel = std::make_unique<Channel>(cc,
                                                 modems_[src_idx].calibration,
                                                 modems_[rcv_idx].calibration,
                                                 transducer_per_modem[src_idx],
                                                 transducer_per_modem[rcv_idx],
                                                 modems_[rcv_idx].receive_boost_db);

        worst_channel_extent = std::max(worst_channel_extent,
                                        channel->write_extent_samples());

        built.push_back({std::move(channel), src_idx, rcv_idx});
    }

    const size_t pair_cap = pair_capacity_for_extent(worst_channel_extent,
                                                     scenario, sample_rate);

    // Pass 2: size one PairBuffer per channel and wire the graph.
    for (auto& bc : built) {
        const double   prop_delay_s = bc.channel->propagation_delay_seconds();
        const uint32_t receiver_fs  = modems_[bc.rcv_idx].sample_rate;

        // Propagation-delay backbone for this directed pair.
        auto pair = std::make_unique<PairBuffer>(
            pair_cap, bc.channel->base_delay_samples());
        PairBuffer* pair_raw = pair.get();
        pair_buffers_.push_back(std::move(pair));

        // Receiver-side fields wired later via wire_modem_trackers; record
        // a back-pointer so the wiring step can find this entry.
        SourceWorker::Outgoing og;
        og.channel              = std::move(bc.channel);
        og.pair_buffer          = pair_raw;
        og.receiver_sample_rate = receiver_fs;
        og.propagation_delay_s  = prop_delay_s;
        per_source_outgoing[bc.src_idx].push_back(std::move(og));
        const size_t outgoing_idx = per_source_outgoing[bc.src_idx].size() - 1;
        per_receiver_outgoing_refs_[bc.rcv_idx].push_back(
            {bc.src_idx, outgoing_idx});

        per_receiver_incoming[bc.rcv_idx].push_back(pair_raw);
    }

    // One SourceWorker per source modem; modems with no outgoing channels
    // get a nullptr slot so receiver_mix indices still line up.
    source_workers_.resize(modems_.size());
    for (size_t i = 0; i < modems_.size(); ++i) {
        if (per_source_outgoing[i].empty()) continue;
        source_workers_[i] = std::make_unique<SourceWorker>(
            modems_[i].id,
            *modems_[i].runtime,
            *modems_[i].tx_ring,
            modems_[i].calibration,
            std::move(per_source_outgoing[i]));
    }

    // One ReceiverMix per modem
    receiver_mixes_.reserve(modems_.size());
    for (size_t i = 0; i < modems_.size(); ++i) {
        receiver_mixes_.push_back(std::make_unique<ReceiverMix>(
            std::move(per_receiver_incoming[i]),
            modems_[i].noise_cfg,
            modems_[i].calibration,
            modems_[i].sample_rate));
    }

    if (!pair_buffers_.empty()) {
        const size_t cap = pair_buffers_.front()->capacity();
        const double cap_s = static_cast<double>(cap) / sample_rate;
        spdlog::info("ChannelEngine: PairBuffer capacity = {} samples "
                     "(= {:.2f} s @ {} kSPS), max_message_duration_s = {:.2f}",
                     cap, cap_s, sample_rate / 1000,
                     scenario.environment.max_message_duration_s);
    }
}

ChannelEngine::~ChannelEngine() {
    stop();
}

void ChannelEngine::start() {
    if (started_.exchange(true, std::memory_order_acq_rel)) return;

    worker_threads_.reserve(source_workers_.size());
    for (auto& sw : source_workers_) {
        if (!sw) {
            worker_threads_.emplace_back();  // empty placeholder
            continue;
        }
        worker_threads_.emplace_back([raw = sw.get()] { raw->run(); });
    }
}

void ChannelEngine::stop() {
    if (!started_.exchange(false, std::memory_order_acq_rel)) return;

    for (auto& sw : source_workers_) {
        if (sw) sw->stop();
    }
    for (auto& t : worker_threads_) {
        if (t.joinable()) t.join();
    }
    worker_threads_.clear();

    // A dropped sample means the PairBuffer was undersized for the
    // scenario's traffic.
    for (size_t i = 0; i < pair_buffers_.size(); ++i) {
        const uint64_t drops = pair_buffers_[i]->overflow_drops();
        if (drops > 0) {
            spdlog::error("ChannelEngine: PairBuffer[{}] dropped {} samples "
                          "— increase environment.max_message_duration_s",
                          i, drops);
        }
    }
}

ReceiverMix* ChannelEngine::receiver_mix(size_t modem_idx) {
    if (modem_idx >= receiver_mixes_.size()) return nullptr;
    return receiver_mixes_[modem_idx].get();
}

void ChannelEngine::set_metrics(Metrics* metrics) {
    for (auto& sw : source_workers_) {
        if (sw) sw->set_metrics(metrics);
    }
}

void ChannelEngine::set_processing_time_stats(ProcessingTimeStats* stats,
                                               uint64_t deadline_us) {
    for (auto& sw : source_workers_) {
        if (!sw) continue;
        sw->set_processing_time_stats(stats);
        sw->set_processing_deadline_us(deadline_us);
    }
}

void ChannelEngine::set_message_event_logs(
    const std::vector<MessageEventLog*>& logs) {
    for (size_t i = 0; i < source_workers_.size() && i < logs.size(); ++i) {
        if (source_workers_[i]) source_workers_[i]->set_message_event_log(logs[i]);
    }
}

SourceWorker* ChannelEngine::source_worker(size_t modem_idx) {
    if (modem_idx >= source_workers_.size()) return nullptr;
    return source_workers_[modem_idx].get();
}

void ChannelEngine::wire_modem_trackers(
    size_t modem_idx,
    IFillTracker* fill_tracker,
    TxStartEstimator* tx_start_estimator,
    SPSCRingBuffer<uint16_t>* rx_ring) {
    if (modem_idx >= modems_.size()) return;

    // If this modem is a source, wire its TxStartEstimator.
    if (modem_idx < source_workers_.size() && source_workers_[modem_idx]) {
        source_workers_[modem_idx]->set_source_tx_estimator(tx_start_estimator);
    }

    // For every channel that feeds INTO this modem (it's the receiver),
    // record its fill_tracker and rx_ring in the source-side outgoing entry.
    for (const auto& ref : per_receiver_outgoing_refs_[modem_idx]) {
        if (ref.source_idx >= source_workers_.size()) continue;
        auto& sw = source_workers_[ref.source_idx];
        if (!sw) continue;
        sw->set_outgoing_receiver_context(ref.outgoing_idx,
                                           fill_tracker, rx_ring);
    }
}

} // namespace openCREST
