#include "logging/stream_logger.hpp"
#include <spdlog/spdlog.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <sys/types.h>
#include <unistd.h>

namespace openCREST::logging {

namespace {

// When running under sudo, hand newly-created files back to the invoking
// user via SUDO_UID / SUDO_GID so they remain editable between runs.
// Best effort — failures are debug-logged, never fatal.
void chown_to_invoking_user(const std::filesystem::path& p) {
    if (::geteuid() != 0) return;
    const char* uid_str = std::getenv("SUDO_UID");
    const char* gid_str = std::getenv("SUDO_GID");
    if (!uid_str || !gid_str) return;
    char* end = nullptr;
    const long uid = std::strtol(uid_str, &end, 10);
    if (end == uid_str || *end != '\0') return;
    end = nullptr;
    const long gid = std::strtol(gid_str, &end, 10);
    if (end == gid_str || *end != '\0') return;
    if (::chown(p.c_str(), static_cast<uid_t>(uid),
                static_cast<gid_t>(gid)) != 0) {
        spdlog::debug("StreamLogger: chown('{}') failed: {}",
                      p.string(), std::strerror(errno));
    }
}

} // namespace

StreamLogger::StreamLogger(const LoggingConfig& config,
                            const std::vector<ModemLogInfo>& modems,
                            uint32_t sample_rate)
    : config_(config)
    , sample_rate_(sample_rate)
{
    if (!config_.log_raw_tx && !config_.log_raw_rx) {
        return;  // Logging disabled; writers stay empty
    }

    if (!config_.output_directory.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(config_.output_directory, ec);
        if (ec) {
            spdlog::warn("StreamLogger: cannot create directory '{}': {}",
                         config_.output_directory, ec.message());
        } else {
            chown_to_invoking_user(config_.output_directory);
        }
    }

    const std::string& dir = config_.output_directory;

    // Wipe .wav files from prior runs so new-session logs aren't mixed with
    // stale data. Restricted to .wav so user artifacts (plots, notes) survive.
    {
        const std::filesystem::path root = dir.empty() ? "." : dir;
        std::error_code ec;
        for (auto it = std::filesystem::directory_iterator(root, ec);
             !ec && it != std::filesystem::directory_iterator();
             it.increment(ec))
        {
            if (!it->is_regular_file()) continue;
            if (it->path().extension() == ".wav") {
                std::error_code rm_ec;
                std::filesystem::remove(it->path(), rm_ec);
            }
        }
    }

    for (const auto& m : modems) {
        if (config_.log_raw_tx) {
            const std::string path = dir.empty()
                ? (m.id + "_tx.wav")
                : (dir + "/" + m.id + "_tx.wav");
            tx_writers_[m.id] = std::make_unique<WavWriter>(
                path, sample_rate, m.adc_bits);
            chown_to_invoking_user(path);
        }
        if (config_.log_raw_rx) {
            rx_source_bits_[m.id] = m.dac_bits;
            rx_session_[m.id] = 1;
            const std::string path = rx_path_for(m.id, 1);
            rx_writers_[m.id] = std::make_unique<WavWriter>(
                path, sample_rate, m.dac_bits);
            chown_to_invoking_user(path);
        }
    }

    spdlog::info("StreamLogger: logging to '{}'", dir.empty() ? "." : dir);
}

StreamLogger::~StreamLogger() {
    finalize();
}

void StreamLogger::log_tx(const std::string& modem_id,
                           const uint16_t* samples, size_t count) {
    auto it = tx_writers_.find(modem_id);
    if (it != tx_writers_.end() && it->second) {
        it->second->write(samples, count);
    }
}

void StreamLogger::log_rx(const std::string& modem_id,
                           const uint16_t* samples, size_t count) {
    auto it = rx_writers_.find(modem_id);
    if (it != rx_writers_.end() && it->second) {
        it->second->write(samples, count);
    }
}

void StreamLogger::begin_rx_session(const std::string& modem_id) {
    if (!config_.log_raw_rx) return;

    auto it = rx_writers_.find(modem_id);
    if (it == rx_writers_.end()) return;

    if (it->second) it->second->finalize();

    const uint32_t next = ++rx_session_[modem_id];
    const uint8_t bits = rx_source_bits_[modem_id];
    const std::string path = rx_path_for(modem_id, next);
    it->second = std::make_unique<WavWriter>(path, sample_rate_, bits);
    chown_to_invoking_user(path);
}

std::string StreamLogger::rx_path_for(const std::string& modem_id,
                                       uint32_t session) const {
    char suffix[32];
    std::snprintf(suffix, sizeof(suffix), "_rx_%03u.wav", session);
    const std::string& dir = config_.output_directory;
    return dir.empty() ? (modem_id + suffix)
                       : (dir + "/" + modem_id + suffix);
}

void StreamLogger::finalize() {
    if (finalized_) return;
    finalized_ = true;

    for (auto& [id, w] : tx_writers_) if (w) w->finalize();
    for (auto& [id, w] : rx_writers_) if (w) w->finalize();
}

} // namespace openCREST::logging
