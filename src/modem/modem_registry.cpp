#include "modem/modem_registry.hpp"
#include "transport/usb_transport.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace openCREST {

ModemRegistry::ModemRegistry() {
    // Default factory: create a real USB transport
    transport_factory = [](const std::string& modem_id) -> std::unique_ptr<IModemTransport> {
        (void)modem_id;
        return std::make_unique<UsbTransport>();
    };
}

std::vector<std::unique_ptr<Modem>> ModemRegistry::discover_and_connect(
    const ScenarioConfig& scenario)
{
    std::vector<std::unique_ptr<Modem>> modems;
    modems.reserve(scenario.modems.size());

    for (const auto& modem_cfg : scenario.modems) {
        spdlog::info("ModemRegistry: connecting modem '{}' (serial: '{}')",
                     modem_cfg.id, modem_cfg.usb_serial);

        auto transport = transport_factory(modem_cfg.usb_serial);
        auto modem = std::make_unique<Modem>(std::move(transport), modem_cfg.usb_serial);

        if (!modem->connect()) {
            throw std::runtime_error(
                "ModemRegistry: cannot connect to modem '" + modem_cfg.id +
                "' (serial: '" + modem_cfg.usb_serial + "')");
        }

        modem->calibrate();
        modem->enter_hil_mode();

        modems.push_back(std::move(modem));
    }

    spdlog::info("ModemRegistry: {} modem(s) connected and ready", modems.size());
    return modems;
}

} // namespace openCREST
