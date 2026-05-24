#include "transport/usb_transport.hpp"
#include "transport/usb_device_discovery.hpp"
#include <spdlog/spdlog.h>
#include <cstring>

namespace openCREST {

UsbTransport::UsbTransport() {
    const int ret = libusb_init(&ctx_);
    if (ret != LIBUSB_SUCCESS) {
        spdlog::error("libusb_init failed: {}", libusb_strerror(static_cast<libusb_error>(ret)));
        ctx_ = nullptr;
    }
}

UsbTransport::~UsbTransport() {
    close();
    if (ctx_) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }
}

bool UsbTransport::open(const std::string& usb_serial) {
    if (!ctx_) return false;
    if (handle_) close();

    libusb_device* dev = find_device_by_serial(ctx_, usb_serial);
    if (!dev) {
        spdlog::error("UsbTransport: modem serial '{}' not found", usb_serial);
        return false;
    }

    const int ret = libusb_open(dev, &handle_);
    libusb_unref_device(dev);

    if (ret != LIBUSB_SUCCESS) {
        spdlog::error("libusb_open failed: {}", libusb_strerror(static_cast<libusb_error>(ret)));
        handle_ = nullptr;
        return false;
    }

    // Detach kernel driver if active (Linux)
    for (int iface : {DATA_INTERFACE_NUM, CTRL_INTERFACE_NUM}) {
        if (libusb_kernel_driver_active(handle_, iface) == 1) {
            libusb_detach_kernel_driver(handle_, iface);
        }
    }

    // Claim both interfaces
    for (int iface : {DATA_INTERFACE_NUM, CTRL_INTERFACE_NUM}) {
        const int claim_ret = libusb_claim_interface(handle_, iface);
        if (claim_ret != LIBUSB_SUCCESS) {
            spdlog::error("libusb_claim_interface({}) failed: {}", iface,
                          libusb_strerror(static_cast<libusb_error>(claim_ret)));
            close();
            return false;
        }
    }

    if (!discover_endpoints()) {
        spdlog::error("UsbTransport: unexpected endpoint layout for serial '{}'", usb_serial);
        close();
        return false;
    }

    // Drain stale bytes left in the device's bulk-IN FIFOs from a prior
    // simulator session; otherwise the first recv_control's 64-byte
    // read can hit LIBUSB_ERROR_OVERFLOW from concatenated status
    // packets.
    drain_in_endpoint(ctrl_ep_in_);
    drain_in_endpoint(data_ep_in_);

    spdlog::info("UsbTransport: opened modem serial '{}'", usb_serial);
    return true;
}

void UsbTransport::close() {
    if (!handle_) return;

    libusb_release_interface(handle_, CTRL_INTERFACE_NUM);
    libusb_release_interface(handle_, DATA_INTERFACE_NUM);
    libusb_close(handle_);
    handle_ = nullptr;
}

bool UsbTransport::is_open() const {
    return handle_ != nullptr;
}

TransferResult UsbTransport::send_data(const uint8_t* data, size_t len, int timeout_ms) {
    return bulk_transfer(data_ep_out_, const_cast<uint8_t*>(data), len, timeout_ms);
}

TransferResult UsbTransport::recv_data(uint8_t* data, size_t len, int timeout_ms) {
    return bulk_transfer(data_ep_in_, data, len, timeout_ms);
}

TransferResult UsbTransport::send_control(const uint8_t* data, size_t len, int timeout_ms) {
    return bulk_transfer(ctrl_ep_out_, const_cast<uint8_t*>(data), len, timeout_ms);
}

TransferResult UsbTransport::recv_control(uint8_t* data, size_t len, int timeout_ms) {
    return bulk_transfer(ctrl_ep_in_, data, len, timeout_ms);
}

TransferResult UsbTransport::bulk_transfer(uint8_t endpoint,
                                             uint8_t* data, size_t len,
                                             int timeout_ms) {
    if (!handle_) {
        return {TransferResult::Status::DISCONNECTED, 0};
    }

    int transferred = 0;
    const int ret = libusb_bulk_transfer(handle_, endpoint, data,
                                          static_cast<int>(len),
                                          &transferred, timeout_ms);
    return map_libusb_error(ret, transferred);
}

void UsbTransport::drain_in_endpoint(uint8_t endpoint) {
    if (!handle_) return;
    // 4 KiB swallows up to 8 back-to-back 512-byte HS bulk packets or
    // 64 back-to-back 64-byte control packets.
    uint8_t buf[4096];
    constexpr int kMaxAttempts    = 16;
    constexpr int kShortTimeoutMs = 5;
    for (int i = 0; i < kMaxAttempts; ++i) {
        int transferred = 0;
        const int ret = libusb_bulk_transfer(handle_, endpoint, buf, sizeof(buf),
                                              &transferred, kShortTimeoutMs);
        if (ret == LIBUSB_ERROR_TIMEOUT) return;   // FIFO drained
        if (ret != LIBUSB_SUCCESS) return;
        if (transferred == 0) return;
    }
}

TransferResult UsbTransport::map_libusb_error(int err, int transferred) const {
    switch (err) {
    case LIBUSB_SUCCESS:
        return {TransferResult::Status::OK, transferred};
    case LIBUSB_ERROR_TIMEOUT:
        return {TransferResult::Status::TIMEOUT, transferred};
    case LIBUSB_ERROR_NO_DEVICE:
    case LIBUSB_ERROR_PIPE:
        return {TransferResult::Status::DISCONNECTED, 0};
    default:
        spdlog::warn("libusb_bulk_transfer error: {}", libusb_strerror(static_cast<libusb_error>(err)));
        return {TransferResult::Status::ERROR, 0};
    }
}

bool UsbTransport::discover_endpoints() {
    // Walk the active configuration. Each of DATA_INTERFACE_NUM and
    // CTRL_INTERFACE_NUM is expected to expose one IN + one OUT bulk
    // endpoint.
    libusb_device* dev = libusb_get_device(handle_);
    libusb_config_descriptor* config = nullptr;
    if (libusb_get_active_config_descriptor(dev, &config) != LIBUSB_SUCCESS) {
        return false;
    }

    bool data_ok = false;
    bool ctrl_ok = false;

    for (uint8_t i = 0; i < config->bNumInterfaces && !(data_ok && ctrl_ok); ++i) {
        const libusb_interface& iface = config->interface[i];
        if (iface.num_altsetting < 1) continue;
        const libusb_interface_descriptor& alts = iface.altsetting[0];

        uint8_t ep_in = 0, ep_out = 0;
        for (uint8_t e = 0; e < alts.bNumEndpoints; ++e) {
            const auto& ep = alts.endpoint[e];
            if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK) {
                continue;
            }
            if (ep.bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                ep_in = ep.bEndpointAddress;
            } else {
                ep_out = ep.bEndpointAddress;
            }
        }

        if (i == static_cast<uint8_t>(DATA_INTERFACE_NUM) && ep_in && ep_out) {
            data_ep_in_  = ep_in;
            data_ep_out_ = ep_out;
            data_ok = true;
        } else if (i == static_cast<uint8_t>(CTRL_INTERFACE_NUM) && ep_in && ep_out) {
            ctrl_ep_in_  = ep_in;
            ctrl_ep_out_ = ep_out;
            ctrl_ok = true;
        }
    }

    libusb_free_config_descriptor(config);
    return data_ok && ctrl_ok;
}

} // namespace openCREST
