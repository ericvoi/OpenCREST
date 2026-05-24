#include "channel/channel_engine.hpp"
#include "channel/geometric_scene.hpp"
#include "core/constants.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace openCREST {

namespace {

struct PairKey {
    size_t channel_idx;
    size_t receiver_idx;
};

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

size_t ChannelEngine::worst_case_pair_capacity(const ScenarioConfig& scenario,
                                                uint32_t sample_rate) {
    const double sound_speed = clamp_sound_speed_engine(
        scenario.environment.sound_speed_m_s);

    // Per-channel max write-offset (base_delay + multipath tail).
    // Static channels: `range_m + MAX_MULTIPATH_DELAY_S`.
    // Geometric channels: longest_path_at_r_max (since Channel anchors
    // base_delay at direct_path_at_r_min and max_tap_delta covers up to
    // longest_path_at_r_max minus that anchor).
    size_t worst_channel_extent = 0;

    for (const auto& cc : scenario.channels) {
        size_t extent = 0;

        if (cc.mode == ChannelMode::Geometric) {
            // Transient scene used only to query worst path length at r_max.
            EnvironmentConfig env;
            env.sound_speed_m_s = scenario.environment.sound_speed_m_s;
            env.saltwater       = scenario.environment.saltwater;
            GeometricScene scene(cc.geometry, env);
            std::array<PathTap, 5> paths_rmax{};
            const std::size_t n = scene.compute_paths(
                cc.geometry.r_max_m, paths_rmax);
            const float longest_at_rmax = (n > 0)
                ? paths_rmax[n - 1].length_m : cc.range_m;
            extent = static_cast<size_t>(std::round(
                static_cast<double>(longest_at_rmax) * sample_rate / sound_speed));
        } else {
            const float r = cc.range_m;
            const size_t base = static_cast<size_t>(std::round(
                static_cast<double>(r) * sample_rate / sound_speed));
            const size_t mp = static_cast<size_t>(std::round(
                MAX_MULTIPATH_DELAY_S * sample_rate));
            extent = base + mp;
        }
        worst_channel_extent = std::max(worst_channel_extent, extent);
    }

    // environment.max_range_m is a floor for the static-style budget;
    // lets scenarios reserve memory for ranges larger than any declared
    // channel. Geometric channels are bounded by their own r_max above.
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
    // while its pull-thread is otherwise idle. Falls back to 10 s.
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
    const size_t   pair_cap    = worst_case_pair_capacity(scenario, sample_rate);

    // Resolve each modem's TransducerSpec. Loader guarantees every
    // transducer_id resolves; test fixtures bypassing the loader fall
    // back to an identity TransducerSpec so the physical-gain math
    // collapses to -TL.
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

    for (const auto& cc : scenario.channels) {
        const size_t src_idx = modem_index(cc.from_modem);
        const size_t rcv_idx = modem_index(cc.to_modem);

        // Build channel first so we can read its base_delay.
        // The receiver's boost (not the source's) applies — boost
        // preserves SNR at the receiver, so signal and noise must scale
        // together.
        auto channel = std::make_unique<Channel>(cc,
                                                  modems_[src_idx].calibration,
                                                  modems_[rcv_idx].calibration,
                                                  transducer_per_modem[src_idx],
                                                  transducer_per_modem[rcv_idx],
                                                  modems_[rcv_idx].receive_boost_db);
        const double prop_delay_s   = channel->propagation_delay_seconds();
        const uint32_t receiver_fs  = modems_[rcv_idx].sample_rate;

        // Propagation-delay backbone for this directed pair.
        auto pair = std::make_unique<PairBuffer>(pair_cap,
                                                  channel->base_delay_samples());
        PairBuffer* pair_raw = pair.get();
        pair_buffers_.push_back(std::move(pair));

        // Receiver-side fields wired later via wire_modem_trackers; record
        // a back-pointer so the wiring step can find this entry.
        SourceWorker::Outgoing og;
        og.channel              = std::move(channel);
        og.pair_buffer          = pair_raw;
        og.receiver_sample_rate = receiver_fs;
        og.propagation_delay_s  = prop_delay_s;
        per_source_outgoing[src_idx].push_back(std::move(og));
        const size_t outgoing_idx = per_source_outgoing[src_idx].size() - 1;
        per_receiver_outgoing_refs_[rcv_idx].push_back({src_idx, outgoing_idx});

        per_receiver_incoming[rcv_idx].push_back(pair_raw);
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

    // One ReceiverMix per modem; a modem with no incoming channels emits
    // silence + noise.
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
    // record its fill_tracker and rx_ring in the source-side Outgoing entry.
    for (const auto& ref : per_receiver_outgoing_refs_[modem_idx]) {
        if (ref.source_idx >= source_workers_.size()) continue;
        auto& sw = source_workers_[ref.source_idx];
        if (!sw) continue;
        sw->set_outgoing_receiver_context(ref.outgoing_idx,
                                           fill_tracker, rx_ring);
    }
}

} // namespace openCREST
