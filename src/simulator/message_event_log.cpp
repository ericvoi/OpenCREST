#include "simulator/message_event_log.hpp"

#include <cstdio>

namespace openCREST {

namespace {

const char* direction_str(MessageEvent::Direction d) {
    return d == MessageEvent::Direction::Tx ? "tx" : "rx";
}

// Escape a string for JSON.
std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

} // namespace

MessageEventLog::~MessageEventLog() { close(); }

bool MessageEventLog::open(const std::string& path) {
    std::lock_guard<std::mutex> lk(mu_);
    if (out_.is_open()) out_.close();
    path_ = path;
    out_.open(path, std::ios::out | std::ios::trunc);
    return out_.is_open();
}

void MessageEventLog::record(const MessageEvent& ev) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!out_.is_open()) return;
    out_ << '{'
         << "\"modem_id\":\""     << json_escape(ev.modem_id) << "\","
         << "\"direction\":\""    << direction_str(ev.direction) << "\","
         << "\"start_ns\":"       << ev.start_ns       << ","
         << "\"end_ns\":"         << ev.end_ns         << ","
         << "\"sample_count\":"   << ev.sample_count   << ","
         << "\"sequence_id\":"    << ev.sequence_id
         << "}\n";
    out_.flush();
}

void MessageEventLog::close() {
    std::lock_guard<std::mutex> lk(mu_);
    if (out_.is_open()) out_.close();
}

} // namespace openCREST
