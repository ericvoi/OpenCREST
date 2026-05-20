#pragma once
#include <string>
#include <cstdint>
#include <cstddef>

namespace openCREST {

// Result of a single USB bulk transfer attempt.
struct TransferResult {
    enum class Status {
        OK,           // Transfer completed successfully
        TIMEOUT,      // No data within timeout_ms
        ERROR,        // USB or I/O error
        DISCONNECTED, // Device was unplugged
    };

    Status status;
    int    bytes_transferred;

    bool ok()           const { return status == Status::OK; }
    bool timed_out()    const { return status == Status::TIMEOUT; }
    bool disconnected() const { return status == Status::DISCONNECTED; }
};

// Abstract interface between I/O threads and the physical (or mock) modem link.
//
// One IModemTransport per modem. The data interface carries 512-byte bulk
// packets; the control interface carries 64-byte bulk packets.
//
// All calls are blocking (synchronous) and return within timeout_ms + small
// driver overhead. Callers must not share one instance across threads.
class IModemTransport {
public:
    virtual ~IModemTransport() = default;

    // Open a connection to the device identified by `device_identifier`.
    // For USB: the USB serial number string.
    // Returns true on success.
    virtual bool open(const std::string& device_identifier) = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;

    // Data interface — 512-byte packets
    virtual TransferResult send_data(const uint8_t* data, size_t len,
                                     int timeout_ms) = 0;
    virtual TransferResult recv_data(uint8_t*       data, size_t len,
                                     int timeout_ms) = 0;

    // Control interface — 64-byte packets
    virtual TransferResult send_control(const uint8_t* data, size_t len,
                                        int timeout_ms) = 0;
    virtual TransferResult recv_control(uint8_t*       data, size_t len,
                                        int timeout_ms) = 0;
};

} // namespace openCREST
