#include "simulator/simulator.hpp"
#include "core/constants.hpp"
#include "dsp/noise_generator.hpp"
#include "dsp/noise_psd.hpp"
#include "dsp/wenz_model.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <stdexcept>
#include <cmath>

namespace openCREST {

using Clock = std::chrono::steady_clock;

namespace {

std::string iso8601_utc(std::chrono::system_clock::time_point t) {
    const std::time_t tt = std::chrono::system_clock::to_time_t(t);
    std::tm utc{};
    gmtime_r(&tt, &utc);
    std::ostringstream oss;
    oss << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

bool run_summary_enabled(const LoggingConfig& cfg) {
    // Either an explicit path override OR any of the new observability
    // flags asks for summary output.
    return !cfg.run_summary_path.empty()
        || cfg.log_processing_time_histogram
        || cfg.log_message_events;
}

std::string join_path(const std::string& dir, const std::string& name) {
    if (dir.empty())          return name;
    if (dir.back() == '/')    return dir + name;
    return dir + "/" + name;
}

} // namespace

Simulator::Simulator(std::string scenario_path)
    : scenario_path_(std::move(scenario_path))
{}

Simulator::~Simulator() {
    stop();
    join_all_threads();
    if (logger_) logger_->finalize();
}

// ---------------------------------------------------------------------------
// initialize()
// ---------------------------------------------------------------------------

bool Simulator::initialize() {
    return load_scenario()
        && discover_modems()
        && calibrate_modems()
        && build_channel_engine();
}

bool Simulator::load_scenario() {
    try {
        scenario_ = ScenarioLoader::load(scenario_path_);
        spdlog::info("Scenario '{}' loaded from '{}'",
                     scenario_.name, scenario_path_);
        return true;
    } catch (const ScenarioLoadError& e) {
        spdlog::error("Failed to load scenario: {}", e.what());
        return false;
    }
}

bool Simulator::discover_modems() {
    try {
        modems_ = registry_.discover_and_connect(scenario_);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Modem discovery failed: {}", e.what());
        return false;
    }
}

bool Simulator::calibrate_modems() {
    // Calibration is performed inside discover_and_connect().
    // Nothing additional to do here; kept as a separate step for clarity
    // and future expansion (e.g., re-calibration during long sessions).
    return true;
}

bool Simulator::build_channel_engine() {
    // Allocate ring buffers (one pair per modem)
    buffers_.resize(modems_.size());

    // Gather sample rate from first modem's calibration
    const uint32_t sample_rate = modems_.empty()
        ? 500'000u
        : modems_[0]->calibration().adc_sampling_rate;

    // Build per-modem contexts for the ChannelEngine
    std::vector<PerModemContext> contexts;
    contexts.reserve(modems_.size());

    // Resolve the rx_atten_idx the simulator must assume per receiver. The
    // hardware-forced index in HIL operation is the same for every channel
    // into a given receiver (the modem switches one global pad), but the
    // YAML lets each channel declare its own. Take the first channel's
    // value as canonical and warn if a sibling disagrees.
    auto rx_atten_idx_for_modem = [&](const std::string& modem_id) -> uint8_t {
        const ChannelConfig* first = nullptr;
        for (const auto& cc : scenario_.channels) {
            if (cc.to_modem != modem_id) continue;
            if (!first) {
                first = &cc;
                continue;
            }
            if (cc.rx_atten_idx != first->rx_atten_idx) {
                spdlog::warn("Receiver '{}' has channels with different "
                             "rx_atten_idx ({} vs {}); using {}",
                             modem_id, first->rx_atten_idx, cc.rx_atten_idx,
                             first->rx_atten_idx);
            }
        }
        return first ? first->rx_atten_idx : uint8_t{1};
    };

    // Pre-design the FIR shaping filter once; its peak-normalised response
    // depends only on (sample_rate, sea_state, saltwater), all scenario-wide.
    const auto shaping_taps = dsp::design_shaping_filter(
        static_cast<float>(sample_rate),
        scenario_.noise.wenz_sea_state,
        scenario_.noise.saltwater,
        32);

    for (size_t i = 0; i < modems_.size(); ++i) {
        const auto& modem = modems_[i];

        // Find the matching ModemConfig by USB serial
        const ModemConfig* cfg = nullptr;
        for (const auto& mc : scenario_.modems) {
            if (mc.usb_serial == modem->id()) {
                cfg = &mc;
                break;
            }
        }

        // Resolve this modem's TransducerSpec from the scenario (loader
        // already validated referential integrity).
        TransducerSpec rx_transducer{};
        if (cfg) {
            const auto it = scenario_.transducers.find(cfg->transducer_id);
            if (it != scenario_.transducers.end()) rx_transducer = it->second;
        }

        const auto& cal = modem->calibration();
        const std::string modem_id = cfg ? cfg->id : modem->id();
        const uint8_t atten_idx = rx_atten_idx_for_modem(modem_id);

        // ------------------------------------------------------------------
        // Per-receiver Phase C noise sizing. Comparison is in the preamp
        // reference frame so it is invariant to the input attenuation pad
        // and to the DAC/ADC voltage references.
        //   1. Wenz amplitude-PSD at fc_rx → V/√Hz at preamp via RVR
        //      (`natural_psd_dbv_at_preamp`).
        //   2. AFE counts/√Hz → V/√Hz at preamp via 1/preamp_gain
        //      (`afe_psd_dbv_at_preamp`). preamp_gain itself is derived
        //      from cal.loopback_gain & cal.input_attenuation[loopback_cal].
        //   3. boost_db = max(0, afe_preamp + margin − natural_preamp).
        //      Fed to every Channel feeding this modem so SNR is preserved
        //      while noise dominates AFE by ≥ margin at the preamp input.
        //   4. (natural + boost)_preamp → DAC sample dBFS by un-doing the
        //      operating input pad and V_ref_dac, then to total-RMS dBFS
        //      for NoiseGenerator via the FIR shape at fc_rx.
        // ------------------------------------------------------------------
        const auto sizing = dsp::compute_receiver_noise_sizing(
            cal, rx_transducer,
            scenario_.noise.wenz_sea_state,
            scenario_.noise.saltwater,
            atten_idx,
            scenario_.noise.min_margin_above_afe_db);

        if (cal.noise_floor_psd_counts_per_sqrt_hz <= 0.0f) {
            spdlog::warn("Modem '{}': AFE noise PSD uncalibrated; using "
                         "fallback {:.1f} dBV/√Hz @ preamp for boost decision",
                         modem_id, dsp::kFallbackAfePsdDbvAtPreamp);
        }
        if (cal.loopback_gain <= 0.0f) {
            spdlog::warn("Modem '{}': loopback_gain not reported (≤ 0); "
                         "preamp gain unknowable, AFE comparison disabled",
                         modem_id);
        }
        const float preamp_gain_db = dsp::preamp_gain_db(cal);
        if (sizing.boost_db > 0.0f) {
            spdlog::warn(
                "Modem '{}': natural ambient PSD ({:.1f} dBV/√Hz @ preamp, "
                "fc={:.1f} kHz) below AFE+margin ({:.1f}+{:.1f} dBV); "
                "boosting receive chain by {:.1f} dB — clipping headroom "
                "drops by the same amount (preamp gain = {:.1f} dB)",
                modem_id,
                sizing.natural_psd_dbv_at_preamp,
                cal.center_freq_hz / 1000.0f,
                sizing.afe_psd_dbv_at_preamp,
                scenario_.noise.min_margin_above_afe_db,
                sizing.boost_db,
                preamp_gain_db);
        } else {
            spdlog::info(
                "Modem '{}': natural ambient PSD={:.1f} dBV/√Hz @ preamp at "
                "fc={:.1f} kHz, AFE={:.1f} dBV/√Hz @ preamp, boost=0 dB "
                "(preamp gain = {:.1f} dB)",
                modem_id,
                sizing.natural_psd_dbv_at_preamp,
                cal.center_freq_hz / 1000.0f,
                sizing.afe_psd_dbv_at_preamp,
                preamp_gain_db);
        }

        dsp::NoiseConfig noise_cfg;
        noise_cfg.wenz_sea_state = scenario_.noise.wenz_sea_state;
        noise_cfg.saltwater      = scenario_.noise.saltwater;
        if (scenario_.noise.disable) {
            // Hard-silence: NoiseGenerator scales to a level so far below
            // full-scale that injection is a no-op in practice.
            noise_cfg.target_level_db_re_fs = -300.0f;
        } else {
            noise_cfg.target_level_db_re_fs = dsp::psd_target_to_total_rms_dbfs(
                sizing.target_psd_dbfs_at_dac,
                shaping_taps,
                cal.center_freq_hz,
                static_cast<float>(sample_rate));
            if (!std::isfinite(noise_cfg.target_level_db_re_fs)) {
                spdlog::warn("Modem '{}': failed to compute target noise "
                             "level (degenerate filter response at fc); "
                             "falling back to silence", modem_id);
                noise_cfg.target_level_db_re_fs = -300.0f;
            }
        }
        for (const auto& ts : scenario_.noise.tonal_sources) {
            noise_cfg.tonals.push_back(ts);
        }

        PerModemContext ctx;
        ctx.id               = modem_id;
        ctx.calibration      = modem->calibration();
        ctx.runtime          = &modem->runtime_state();
        ctx.tx_ring          = &buffers_[i].tx_ring;
        ctx.rx_ring          = &buffers_[i].rx_ring;
        ctx.noise_cfg        = std::move(noise_cfg);
        ctx.sample_rate      = sample_rate;
        ctx.receive_boost_db = sizing.boost_db;
        contexts.push_back(std::move(ctx));
    }

    engine_ = std::make_unique<ChannelEngine>(scenario_, std::move(contexts));
    engine_->set_metrics(&metrics_);
    spdlog::info("ChannelEngine built with {} channel(s)", scenario_.channels.size());

    // Build the stream logger
    std::vector<logging::ModemLogInfo> log_infos;
    log_infos.reserve(modems_.size());
    for (const auto& m : modems_) {
        const auto& c = m->calibration();
        log_infos.push_back({m->id(), c.adc_bits, c.dac_bits});
    }
    logger_ = std::make_unique<logging::StreamLogger>(
        scenario_.logging, log_infos, sample_rate);

    return true;
}

// ---------------------------------------------------------------------------
// run()
// ---------------------------------------------------------------------------

void Simulator::install_observability() {
    const auto& log = scenario_.logging;

    // Processing-time histogram: one shared instance across all source
    // workers. Deadline derived from the processing block size at the
    // first modem's sample rate (all modems share a single rate today).
    if (log.log_processing_time_histogram) {
        processing_time_stats_ = std::make_unique<ProcessingTimeStats>();
        const uint32_t fs = modems_.empty()
            ? 500'000u : modems_[0]->calibration().adc_sampling_rate;
        const uint64_t deadline_us = static_cast<uint64_t>(
            (static_cast<double>(PROCESSING_BLOCK_SIZE) /
             static_cast<double>(fs)) * 1e6);
        if (engine_) {
            engine_->set_processing_time_stats(processing_time_stats_.get(),
                                                deadline_us);
        }
    }

    // Per-source-modem message-event JSONL log.
    if (log.log_message_events) {
        message_event_logs_.resize(modems_.size());
        std::vector<MessageEventLog*> raw_logs(modems_.size(), nullptr);
        for (size_t i = 0; i < modems_.size(); ++i) {
            auto el = std::make_unique<MessageEventLog>();
            const std::string path = join_path(
                log.output_directory, modems_[i]->id() + "_events.jsonl");
            if (!el->open(path)) {
                spdlog::warn("Could not open message event log '{}'", path);
            } else {
                raw_logs[i] = el.get();
            }
            message_event_logs_[i] = std::move(el);
        }
        if (engine_) engine_->set_message_event_logs(raw_logs);
    }
}

void Simulator::emit_run_summary() {
    if (!run_summary_enabled(scenario_.logging)) return;

    RunSummary s;
    s.scenario_name = scenario_.name;
    s.scenario_path = scenario_path_;
    s.random_seed   = scenario_.random_seed;
    s.started_at    = iso8601_utc(run_started_at_);
    s.ended_at      = iso8601_utc(std::chrono::system_clock::now());
    s.duration_s    = std::chrono::duration<double>(
        Clock::now() - run_started_steady_).count();
    s.modems.reserve(modems_.size());
    for (const auto& m : modems_) s.modems.push_back(m->id());

    if (processing_time_stats_) {
        s.processing_time = processing_time_stats_->snapshot();
    }
    s.channel_engine = snapshot_engine_counters(metrics_);

    for (const auto& el : message_event_logs_) {
        if (el && el->is_open()) s.log_files.events.push_back(el->path());
    }

    const std::string path = resolve_run_summary_path(scenario_);
    if (!write_run_summary_json(s, path)) {
        spdlog::warn("Failed to write run summary to '{}'", path);
    } else {
        spdlog::info("Run summary written to '{}'", path);
    }
}

void Simulator::run() {
    running_.store(true, std::memory_order_relaxed);
    run_started_at_     = std::chrono::system_clock::now();
    run_started_steady_ = Clock::now();

    install_observability();

    // Enter HIL mode on all modems
    for (auto& modem : modems_) {
        try {
            modem->enter_hil_mode();
            // The modem is now in RX, but it only sends status packets in
            // response to received data.  Bootstrap the I/O loop by setting
            // the host-side state to RX so send_rx_data() starts immediately.
            modem->runtime_state().state.store(ModemState::RX,
                                               std::memory_order_release);
        } catch (const std::exception& e) {
            spdlog::error("Failed to enter HIL mode on '{}': {}", modem->id(), e.what());
            stop();
            return;
        }
    }

    // Spawn the per-source SourceWorker threads (owned by the engine).
    if (engine_) engine_->start();

    start_io_threads();

    // Main thread: print metrics every second, watch for stop signal
    spdlog::info("Simulation running. Press Ctrl+C to stop.");
    auto interval_start = Clock::now();

    while (running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!running_.load(std::memory_order_relaxed)) break;

        const auto now = Clock::now();
        const double interval_s =
            std::chrono::duration<double>(now - interval_start).count();
        interval_start = now;
        metrics_.print_summary(interval_s);
        metrics_.reset();
    }

    join_all_threads();

    // Stop & join the SourceWorker threads inside the engine.
    if (engine_) engine_->stop();

    // Exit HIL mode
    for (auto& modem : modems_) {
        try { modem->exit_hil_mode(); } catch (...) {}
    }

    if (logger_) logger_->finalize();

    // Close per-modem event logs before composing the summary so the
    // events.jsonl files referenced from the summary are flushed.
    for (auto& el : message_event_logs_) {
        if (el) el->close();
    }

    emit_run_summary();

    spdlog::info("Simulation stopped.");
}

void Simulator::stop() {
    running_.store(false, std::memory_order_relaxed);
    for (auto& worker : io_workers_) {
        if (worker) worker->stop();
    }
    if (engine_) engine_->stop();
}

// ---------------------------------------------------------------------------
// Private thread management
// ---------------------------------------------------------------------------

void Simulator::start_io_threads() {
    io_workers_.reserve(modems_.size());
    io_threads_.reserve(modems_.size());

    for (size_t i = 0; i < modems_.size(); ++i) {
        ReceiverMix* rx_mix = engine_ ? engine_->receiver_mix(i) : nullptr;
        auto worker = std::make_unique<ModemIO>(
            *modems_[i],
            buffers_[i].tx_ring,
            buffers_[i].rx_ring,
            rx_mix,
            logger_.get(),
            &metrics_
        );
        io_workers_.push_back(std::move(worker));

        // Hand the freshly-built tracker + tx-start estimator to the
        // ChannelEngine so SourceWorker arrival-alignment can query
        // them across modem pairs. Safe to call before run() spawns
        // any thread.
        if (engine_) {
            engine_->wire_modem_trackers(i,
                &io_workers_[i]->fill_tracker(),
                &io_workers_[i]->tx_start_estimator(),
                &buffers_[i].rx_ring);
        }

        io_threads_.emplace_back([this, i] {
            io_workers_[i]->run();
        });
    }
}

void Simulator::join_all_threads() {
    for (auto& t : io_threads_) {
        if (t.joinable()) t.join();
    }
    io_threads_.clear();
}

} // namespace openCREST
