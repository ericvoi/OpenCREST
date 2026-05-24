#pragma once
#include "transport/modem_transport.hpp"
#include "core/types.hpp"
#include "protocol/packets.hpp"
#include <array>
#include <deque>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <functional>

namespace openCREST {

// Queue-based deterministic mock transport for unit and integration tests.
//
// Simulates one modem without USB hardware. Test code pre-loads a TX
// waveform, configures calibration, and optionally scripts timed state
// transitions; the mock responds to host commands as a real modem
// would.
//
// Simulated clock advances by `timeout_ms` on each recv_control() call,
// so scripted transitions fire without wall-clock delays.
class MockTransport : public IModemTransport {
public:
    using DataPacket = std::array<uint8_t, protocol::DATA_PACKET_BYTES>;

    // Calibration data returned in response to REQUEST_CALIBRATION.
    void set_calibration(const CalibrationData& cal);

    // Enqueue a pre-encoded 512-byte TX packet (modem → host). When the
    // queue drains while state == TX, the mock auto-transitions to
    // SETTLING.
    void enqueue_tx_packet(const DataPacket& pkt);

    // Encode `samples` into 512-byte packets and enqueue them.
    void enqueue_tx_waveform(const std::vector<uint16_t>& samples);

    // Schedule a state transition at `delay_ms` from the current
    // simulated time. Fires during the next recv_control() that passes
    // the scheduled time.
    void schedule_state_transition(uint64_t delay_ms, ModemState next_state);

    // Set initial state (default IDLE). Must be called before open().
    void set_initial_state(ModemState state);

    // Settling period for auto TX→SETTLING→RX transitions (default 200 ms).
    void set_settling_time_ms(uint64_t ms);

    // All RX packets received via send_data(), in order.
    const std::vector<DataPacket>& received_rx_packets() const;

    // Simulated wall-clock in ms (advanced by recv_control).
    uint64_t simulated_time_ms() const;

    ModemState current_state() const;

    // IModemTransport
    bool open(const std::string& device_identifier) override;
    void close() override;
    bool is_open() const override;

    // Capture RX packet (host → modem).
    TransferResult send_data(const uint8_t* data, size_t len, int timeout_ms) override;
    // Dequeue next TX packet (modem → host); TIMEOUT if state != TX or
    // queue empty.
    TransferResult recv_data(uint8_t* data, size_t len, int timeout_ms) override;

    // Process host commands (calibration request, HIL mode, ...).
    TransferResult send_control(const uint8_t* data, size_t len, int timeout_ms) override;
    // Return a calibration response (if pending) or a status packet.
    // Advances the simulated clock and fires scheduled transitions.
    TransferResult recv_control(uint8_t* data, size_t len, int timeout_ms) override;

private:
    void apply_transitions();

    bool       is_open_             = false;
    ModemState state_               = ModemState::IDLE;
    uint64_t   sim_time_ms_         = 0;
    uint64_t   settling_time_ms_    = 200;

    CalibrationData calibration_{};
    bool            pending_calibration_ = false;
    uint8_t         attenuation_idx_     = 0;
    uint32_t        error_flags_         = 0;
    uint16_t        rx_expected_id_      = 0;  // Next expected host→modem packet_id

    // TX waveform queue (modem → host)
    std::deque<DataPacket> tx_queue_;

    // RX packets captured from host
    std::vector<DataPacket> rx_packets_;

    // Scheduled state transitions: {absolute_sim_time_ms, new_state}
    struct Transition { uint64_t time_ms; ModemState state; };
    std::vector<Transition> transitions_;
};

} // namespace openCREST
