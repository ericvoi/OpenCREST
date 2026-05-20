#pragma once
#include "modem/modem.hpp"
#include "config/scenario.hpp"
#include <functional>
#include <memory>
#include <vector>
#include <string>

namespace openCREST {

// Discovers USB modems listed in the scenario and constructs Modem objects.
//
// Transport creation is delegated to `transport_factory`, which defaults to
// creating UsbTransport instances. Override the factory in tests to inject
// MockTransport without changing any other code:
//
//   ModemRegistry registry;
//   registry.transport_factory = [](const std::string& id) {
//       auto mock = std::make_unique<MockTransport>();
//       mock->set_calibration(test_cal);
//       return mock;
//   };
//   auto modems = registry.discover_and_connect(scenario);
class ModemRegistry {
public:
    ModemRegistry();

    // Factory callable: receives modem_id (USB serial), returns transport.
    // Defaults to constructing a UsbTransport.
    std::function<std::unique_ptr<IModemTransport>(const std::string& modem_id)>
        transport_factory;

    // Discover and connect all modems listed in the scenario.
    // For each ModemConfig in scenario.modems:
    //   - Creates a transport via transport_factory(modem.usb_serial)
    //   - Calls Modem::connect() + calibrate() + enter_hil_mode()
    //   - Appends to the returned vector
    //
    // Throws std::runtime_error if any modem cannot be connected or calibrated.
    std::vector<std::unique_ptr<Modem>> discover_and_connect(
        const ScenarioConfig& scenario);
};

} // namespace openCREST
