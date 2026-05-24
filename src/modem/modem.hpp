#pragma once
#include "transport/modem_transport.hpp"
#include "core/types.hpp"
#include <memory>
#include <string>

namespace openCREST {

// Represents a single physical modem.
//
// Wraps an IModemTransport to drive the protocol handshake (calibration
// exchange, HIL mode entry) and expose shared runtime state to the I/O
// thread and simulator coordinator.
//
// Lifecycle:
//   1. connect()        — open the transport
//   2. calibrate()      — exchange calibration data
//   3. enter_hil_mode() — put modem in HIL streaming mode
//   4. (simulation runs, I/O thread reads/writes via transport())
//   5. exit_hil_mode()  — return modem to normal operation
//
// Lifecycle methods run on the main thread before the I/O thread starts.
// ModemRuntimeState atomics make runtime_state() safe for cross-thread use.
class Modem {
public:
    Modem(std::unique_ptr<IModemTransport> transport,
          const std::string& modem_id);

    ~Modem();

    Modem(const Modem&)            = delete;
    Modem& operator=(const Modem&) = delete;

    // Open the transport. Returns false on failure.
    bool connect();

    // Send REQUEST_CALIBRATION and parse the response. Result is also stored
    // in calibration(). Throws on timeout or protocol error.
    CalibrationData calibrate();

    void enter_hil_mode();
    void exit_hil_mode();
    void select_attenuation(uint8_t index);

    const std::string&     id()            const { return id_; }
    const CalibrationData& calibration()   const { return calibration_; }

    IModemTransport& transport() { return *transport_; }

    // Shared state, written by I/O thread, read by simulator/coordinator.
    ModemRuntimeState& runtime_state() { return runtime_state_; }

private:
    std::unique_ptr<IModemTransport> transport_;
    std::string         id_;
    CalibrationData     calibration_{};
    ModemRuntimeState   runtime_state_;
};

} // namespace openCREST
