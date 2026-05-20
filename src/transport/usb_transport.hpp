#pragma once
#include "transport/modem_transport.hpp"
#include <libusb-1.0/libusb.h>
#include <cstdint>

namespace openCREST {

// USB bulk transport using libusb-1.0 synchronous transfers.
//
// One instance per modem. Each instance owns its own libusb_context for
// thread-safety isolation (one I/O thread per modem).
//
// Endpoint addresses are discovered from the USB device descriptor at open().
// The HIL modem firmware is expected to expose two bulk interfaces:
//   Interface 0: data (512-byte packets)
//   Interface 1: control (64-byte packets)
class UsbTransport : public IModemTransport {
public:
    UsbTransport();
    ~UsbTransport() override;

    // Non-copyable, non-movable (owns libusb handles)
    UsbTransport(const UsbTransport&)            = delete;
    UsbTransport& operator=(const UsbTransport&) = delete;

    // Open by USB serial number. Returns false if device not found or cannot
    // be opened. Endpoint addresses are discovered from descriptors.
    bool open(const std::string& usb_serial) override;
    void close() override;
    bool is_open() const override;

    TransferResult send_data(const uint8_t* data, size_t len, int timeout_ms) override;
    TransferResult recv_data(uint8_t*       data, size_t len, int timeout_ms) override;
    TransferResult send_control(const uint8_t* data, size_t len, int timeout_ms) override;
    TransferResult recv_control(uint8_t*       data, size_t len, int timeout_ms) override;

private:
    // Discover and cache endpoint addresses from the device's interface
    // descriptors. Returns false if the expected interface/endpoint layout
    // is not found.
    bool discover_endpoints();

    TransferResult bulk_transfer(uint8_t endpoint,
                                 uint8_t* data, size_t len,
                                 int timeout_ms);

    // Drain any bytes the device has queued on an IN endpoint from a prior
    // session. Without this, the first bulk_transfer can return
    // LIBUSB_ERROR_OVERFLOW if multiple status packets are concatenated
    // beyond the caller's buffer.
    void drain_in_endpoint(uint8_t endpoint);

    TransferResult map_libusb_error(int err, int transferred) const;

    libusb_context*       ctx_    = nullptr;
    libusb_device_handle* handle_ = nullptr;

    // Bulk endpoint addresses (filled by discover_endpoints())
    uint8_t data_ep_in_  = 0x83;  // modem → host (data interface)
    uint8_t data_ep_out_ = 0x03;  // host → modem (data interface)
    uint8_t ctrl_ep_in_  = 0x84;  // modem → host (control interface)
    uint8_t ctrl_ep_out_ = 0x04;  // host → modem (control interface)

    // Interface numbers (claimed at open, released at close)
    // Interfaces 0–1 are CDC ACM (serial port); HIL uses interfaces 2–3.
    static constexpr int DATA_INTERFACE_NUM = 2;
    static constexpr int CTRL_INTERFACE_NUM = 3;
};

} // namespace openCREST
