#pragma once
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace openCREST {

struct MessageEvent {
    enum class Direction { Tx, Rx };

    std::string  modem_id;
    Direction    direction;
    uint64_t     start_ns;      // steady_clock at TX-entry
    uint64_t     end_ns;        // steady_clock at TX-exit
    uint64_t     sample_count;  // samples consumed during this message
    uint64_t     sequence_id;   // host-assigned, monotonic per (modem, direction)
};

// One MessageEventLog per modem; the simulator opens one per source
// modem and forwards events from SourceWorker on TX-entry / TX-exit
// edges.
//
// JSONL output: one object per line, flushed on every record() so a
// killed run still has up-to-the-last-message data.
class MessageEventLog {
public:
    MessageEventLog() = default;
    ~MessageEventLog();

    MessageEventLog(const MessageEventLog&)            = delete;
    MessageEventLog& operator=(const MessageEventLog&) = delete;

    // Open the JSONL file for append. Returns false on filesystem error;
    // the caller may log and continue with logging disabled.
    bool open(const std::string& path);

    // Append a single event line. Safe to call concurrently — the file
    // mutex serializes writes. No-op if the log was never opened.
    void record(const MessageEvent& ev);

    void close();

    bool is_open() const { return out_.is_open(); }
    const std::string& path() const { return path_; }

private:
    std::ofstream out_;
    std::mutex    mu_;
    std::string   path_;
};

} // namespace openCREST
