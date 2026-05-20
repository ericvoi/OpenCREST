#pragma once
#include "transport/modem_transport.hpp"
#include "core/types.hpp"
#include <memory>
#include <string>

namespace openCREST {

// Represents a single physical modem.
//
// Wraps an IModemTransport to manage the protocol handshake (calibration
// exchange, HIL mode entry) and expose shared runtime state to the I/O
// thread and the simulator coordinator.
//
// Lifecycle:
//   1. connect()        — open the transport
//   2. calibrate()      — exchange calibration data
//   3. enter_hil_mode() — put modem in HIL streaming mode
//   4. (simulation runs, I/O thread reads/writes via transport())
//   5. exit_hil_mode()  — return modem to normal operation
//
// All lifecycle methods are called from the main thread before the I/O
// thread is started. Thread-safety of runtime_state() fields is guaranteed
// by the atomic members of ModemRuntimeState.
class Modem {
public:
    Modem(std::unique_ptr<IModemTransport> transport,
          const std::string& modem_id);

    ~Modem();

    // Non-copyable, non-movable (unique_ptr + atomics)
    Modem(const Modem&)            = delete;
    Modem& operator=(const Modem&) = delete;

    // -----------------------------------------------------------------------
    // Lifecycle (main thread)
    // -----------------------------------------------------------------------

    // Open the transport. Returns false on failure.
    bool connect();

    // Send REQUEST_CALIBRATION command and parse the response.
    // Returns the received CalibrationData; also accessible via calibration().
    // Throws std::runtime_error on timeout or protocol error.
    CalibrationData calibrate();

    // Send ENTER_HIL_MODE command.
    void enter_hil_mode();

    // Send EXIT_HIL_MODE command.
    void exit_hil_mode();

    // Send SELECT_ATTENUATION command.
    void select_attenuation(uint8_t index);

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    const std::string&     id()            const { return id_; }
    const CalibrationData& calibration()   const { return calibration_; }

    // Direct transport access for the I/O thread.
    IModemTransport& transport() { return *transport_; }

    // Shared state (written by I/O thread, read by simulator/coordinator).
    ModemRuntimeState& runtime_state() { return runtime_state_; }

private:
    std::unique_ptr<IModemTransport> transport_;
    std::string         id_;
    CalibrationData     calibration_{};
    ModemRuntimeState   runtime_state_;
};

} // namespace openCREST
