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
// Simulates a single modem without USB hardware. Test code pre-loads a
// TX waveform (as encoded 512-byte packets), configures the calibration
// response, and optionally scripts timed state transitions. The mock
// responds to host commands exactly as a real modem would.
//
// Simulated clock: advanced by `timeout_ms` on each recv_control() call,
// allowing state-transition scripts to fire without wall-clock delays.
class MockTransport : public IModemTransport {
public:
    using DataPacket = std::array<uint8_t, protocol::DATA_PACKET_BYTES>;

    // -----------------------------------------------------------------------
    // Test setup API
    // -----------------------------------------------------------------------

    // Set the calibration data returned in response to REQUEST_CALIBRATION.
    void set_calibration(const CalibrationData& cal);

    // Enqueue pre-encoded 512-byte TX packets (modem → host direction).
    // Once all packets are consumed while state == TX, the mock transitions
    // to SETTLING automatically.
    void enqueue_tx_packet(const DataPacket& pkt);

    // Convenience: encode `samples` into as many 512-byte packets as needed
    // and enqueue them all.
    void enqueue_tx_waveform(const std::vector<uint16_t>& samples);

    // Schedule a state transition at a simulated time offset from now.
    // Transitions fire during the next recv_control() call that advances
    // past the scheduled time.
    void schedule_state_transition(uint64_t delay_ms, ModemState next_state);

    // Set initial modem state (default: IDLE). Must be called before open().
    void set_initial_state(ModemState state);

    // Set the settling period (default: 200 ms) used for automatic
    // TX→SETTLING→RX transitions.
    void set_settling_time_ms(uint64_t ms);

    // -----------------------------------------------------------------------
    // Verification API
    // -----------------------------------------------------------------------

    // Returns all RX packets received via send_data(), in order.
    const std::vector<DataPacket>& received_rx_packets() const;

    // Simulated wall-clock time in milliseconds (advanced by recv_control).
    uint64_t simulated_time_ms() const;

    ModemState current_state() const;

    // -----------------------------------------------------------------------
    // IModemTransport
    // -----------------------------------------------------------------------

    bool open(const std::string& device_identifier) override;
    void close() override;
    bool is_open() const override;

    // Data interface
    // send_data: capture RX packet (host → modem).
    TransferResult send_data(const uint8_t* data, size_t len, int timeout_ms) override;
    // recv_data: dequeue next TX packet (modem → host). Returns TIMEOUT if
    //            state != TX or the TX queue is empty.
    TransferResult recv_data(uint8_t* data, size_t len, int timeout_ms) override;

    // Control interface
    // send_control: process host commands (calibration request, HIL mode, etc.).
    TransferResult send_control(const uint8_t* data, size_t len, int timeout_ms) override;
    // recv_control: return a calibration response (if pending) or status packet.
    //               Advances the simulated clock and fires scheduled transitions.
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
