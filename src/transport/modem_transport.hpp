#pragma once
#include <string>
#include <cstdint>
#include <cstddef>

namespace openCREST {

// Result of a single bulk transfer attempt.
struct TransferResult {
    enum class Status {
        OK,
        TIMEOUT,      // No data within timeout_ms
        ERROR,
        DISCONNECTED, // Device unplugged
    };

    Status status;
    int    bytes_transferred;

    bool ok()           const { return status == Status::OK; }
    bool timed_out()    const { return status == Status::TIMEOUT; }
    bool disconnected() const { return status == Status::DISCONNECTED; }
};

// Abstract interface between I/O threads and the physical (or mock)
// modem link.
//
// One instance per modem. Data interface uses 512-byte bulk packets;
// control interface uses 64-byte bulk packets. All calls block until
// completion or timeout_ms (plus small driver overhead). Instances are
// not thread-safe.
class IModemTransport {
public:
    virtual ~IModemTransport() = default;

    // `device_identifier`: USB serial number (for UsbTransport).
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
