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
// Transport creation is delegated to `transport_factory`, defaulting to
// UsbTransport. Override the factory in tests to inject MockTransport:
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

    // Receives modem_id (USB serial), returns a transport. Defaults to
    // constructing a UsbTransport.
    std::function<std::unique_ptr<IModemTransport>(const std::string& modem_id)>
        transport_factory;

    // For each ModemConfig in scenario.modems: build a transport, then
    // connect + calibrate + enter HIL mode. Throws on any failure.
    std::vector<std::unique_ptr<Modem>> discover_and_connect(
        const ScenarioConfig& scenario);
};

} // namespace openCREST
