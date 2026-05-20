#include "transport/usb_device_discovery.hpp"
#include <spdlog/spdlog.h>

namespace openCREST {

std::string read_string_descriptor(libusb_context* ctx,
                                    libusb_device*  dev,
                                    uint8_t         index) {
    if (index == 0) return {};

    libusb_device_handle* handle = nullptr;
    if (libusb_open(dev, &handle) != LIBUSB_SUCCESS) {
        return {};
    }

    unsigned char buf[256] = {};
    const int ret = libusb_get_string_descriptor_ascii(handle, index, buf, sizeof(buf) - 1);
    libusb_close(handle);

    if (ret < 0) return {};
    return std::string(reinterpret_cast<char*>(buf), static_cast<size_t>(ret));
}

std::vector<UsbDeviceInfo> discover_hil_modems(libusb_context* ctx) {
    std::vector<UsbDeviceInfo> result;

    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(ctx, &list);
    if (count < 0) {
        spdlog::error("libusb_get_device_list failed: {}", libusb_strerror(static_cast<libusb_error>(count)));
        return result;
    }

    for (ssize_t i = 0; i < count; ++i) {
        libusb_device* dev = list[i];
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(dev, &desc) != LIBUSB_SUCCESS) {
            continue;
        }
        if (desc.idVendor != HIL_MODEM_VENDOR_ID || desc.idProduct != HIL_MODEM_PRODUCT_ID) {
            continue;
        }

        const std::string serial = read_string_descriptor(ctx, dev, desc.iSerialNumber);
        UsbDeviceInfo info;
        info.serial_number = serial;
        info.bus           = libusb_get_bus_number(dev);
        info.port          = libusb_get_port_number(dev);
        result.push_back(std::move(info));

        spdlog::debug("Found HIL modem: serial={} bus={} port={}", serial,
                      static_cast<int>(info.bus), static_cast<int>(info.port));
    }

    libusb_free_device_list(list, 1);
    return result;
}

libusb_device* find_device_by_serial(libusb_context*    ctx,
                                      const std::string& serial_number) {
    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(ctx, &list);
    if (count < 0) {
        spdlog::error("libusb_get_device_list failed: {}", libusb_strerror(static_cast<libusb_error>(count)));
        return nullptr;
    }

    libusb_device* found = nullptr;
    for (ssize_t i = 0; i < count; ++i) {
        libusb_device* dev = list[i];
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(dev, &desc) != LIBUSB_SUCCESS) {
            continue;
        }
        if (desc.idVendor != HIL_MODEM_VENDOR_ID || desc.idProduct != HIL_MODEM_PRODUCT_ID) {
            continue;
        }

        const std::string serial = read_string_descriptor(ctx, dev, desc.iSerialNumber);
        if (serial == serial_number) {
            libusb_ref_device(dev);
            found = dev;
            break;
        }
    }

    libusb_free_device_list(list, 1);
    return found;
}

} // namespace openCREST
