#pragma once
#include "transport/modem_transport.hpp"
#include <libusb-1.0/libusb.h>
#include <cstdint>

namespace openCREST {

// USB bulk transport using libusb-1.0 synchronous transfers.
//
// One instance per modem; each owns its own libusb_context for
// per-modem thread isolation. Endpoint addresses are discovered from
// the device descriptor at open().
class UsbTransport : public IModemTransport {
public:
    UsbTransport();
    ~UsbTransport() override;

    UsbTransport(const UsbTransport&)            = delete;
    UsbTransport& operator=(const UsbTransport&) = delete;

    // Open by USB serial number. False if not found or open fails.
    bool open(const std::string& usb_serial) override;
    void close() override;
    bool is_open() const override;

    TransferResult send_data(const uint8_t* data, size_t len, int timeout_ms) override;
    TransferResult recv_data(uint8_t*       data, size_t len, int timeout_ms) override;
    TransferResult send_control(const uint8_t* data, size_t len, int timeout_ms) override;
    TransferResult recv_control(uint8_t*       data, size_t len, int timeout_ms) override;

private:
    // Cache endpoint addresses from interface descriptors. False if the
    // expected layout isn't found.
    bool discover_endpoints();

    TransferResult bulk_transfer(uint8_t endpoint,
                                 uint8_t* data, size_t len,
                                 int timeout_ms);

    // Drain stale bytes queued on an IN endpoint from a prior session.
    // Required to avoid LIBUSB_ERROR_OVERFLOW when multiple status
    // packets sit concatenated in the device FIFO.
    void drain_in_endpoint(uint8_t endpoint);

    TransferResult map_libusb_error(int err, int transferred) const;

    libusb_context*       ctx_    = nullptr;
    libusb_device_handle* handle_ = nullptr;

    // Bulk endpoint addresses (filled by discover_endpoints()).
    uint8_t data_ep_in_  = 0x83;  // modem → host (data interface)
    uint8_t data_ep_out_ = 0x03;  // host → modem (data interface)
    uint8_t ctrl_ep_in_  = 0x84;  // modem → host (control interface)
    uint8_t ctrl_ep_out_ = 0x04;  // host → modem (control interface)

    // Interfaces 0–1 are CDC ACM (serial port); HIL uses 2–3.
    static constexpr int DATA_INTERFACE_NUM = 2;
    static constexpr int CTRL_INTERFACE_NUM = 3;
};

} // namespace openCREST
