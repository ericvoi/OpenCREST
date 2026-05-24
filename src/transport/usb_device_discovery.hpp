#pragma once
#include <libusb-1.0/libusb.h>
#include <string>
#include <vector>

namespace openCREST {

// Vendor / product IDs for the HIL modem USB device.
constexpr uint16_t HIL_MODEM_VENDOR_ID  = 0xCafe;  // TODO: assign real VID
constexpr uint16_t HIL_MODEM_PRODUCT_ID = 0x4001;  // TODO: assign real PID

struct UsbDeviceInfo {
    std::string serial_number;
    uint8_t     bus;
    uint8_t     port;
};

// Enumerate all attached HIL modems. `ctx` must be a valid libusb context.
std::vector<UsbDeviceInfo> discover_hil_modems(libusb_context* ctx);

// libusb_device matching `serial_number`, or nullptr. Returned device
// has its reference count incremented; caller must libusb_unref_device()
// when done (UsbTransport::open() does this internally).
libusb_device* find_device_by_serial(libusb_context*    ctx,
                                      const std::string& serial_number);

// Read the ASCII string descriptor at `index`; empty string on failure.
std::string read_string_descriptor(libusb_context* ctx,
                                    libusb_device*  dev,
                                    uint8_t         index);

} // namespace openCREST
