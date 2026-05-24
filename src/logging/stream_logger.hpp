#pragma once
#include "logging/wav_writer.hpp"
#include "config/scenario.hpp"
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace openCREST::logging {

struct ModemLogInfo {
    std::string id;
    uint8_t adc_bits;   // source bit depth for TX WAV (modem → host)
    uint8_t dac_bits;   // source bit depth for RX WAV (host → modem)
};

// Per-stream WAV logger, one file per modem per direction.
//
// Called from I/O threads; not internally thread-safe — each modem's I/O
// thread owns its own log_tx / log_rx / begin_rx_session calls.
//
// File names:
//   TX (single file per run):  {output_dir}/{modem_id}_tx.wav
//   RX (one file per session): {output_dir}/{modem_id}_rx_{NNN}.wav
//
// An RX "session" is one continuous RX-state stretch. begin_rx_session()
// finalizes the prior RX file and opens the next index. NNN=001 is opened
// at construction so the initial RX phase (before any TX) is logged.
//
// No-op when both log_raw_tx and log_raw_rx are false.
class StreamLogger {
public:
    StreamLogger(const LoggingConfig& config,
                 const std::vector<ModemLogInfo>& modems,
                 uint32_t sample_rate);
    ~StreamLogger();

    // Modem → host samples. No-op if log_raw_tx is off or modem_id unknown.
    void log_tx(const std::string& modem_id,
                const uint16_t* samples, size_t count);

    // Host → modem samples. No-op if log_raw_rx is off or modem_id unknown.
    void log_rx(const std::string& modem_id,
                const uint16_t* samples, size_t count);

    // Finalize the current RX WAV and open the next one (NNN+1). Call on
    // every RX-state entry after the first. No-op if log_raw_rx is off or
    // modem_id is unknown.
    void begin_rx_session(const std::string& modem_id);

    // Patch headers on all WAV files. Idempotent.
    void finalize();

private:
    std::string rx_path_for(const std::string& modem_id, uint32_t session) const;

    LoggingConfig config_;
    uint32_t      sample_rate_ = 0;
    std::unordered_map<std::string, std::unique_ptr<WavWriter>> tx_writers_;
    std::unordered_map<std::string, std::unique_ptr<WavWriter>> rx_writers_;
    std::unordered_map<std::string, uint32_t>                   rx_session_;
    std::unordered_map<std::string, uint8_t>                    rx_source_bits_;
    bool finalized_ = false;
};

} // namespace openCREST::logging
