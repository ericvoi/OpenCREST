#include "transport/mock_transport.hpp"
#include "protocol/protocol_codec.hpp"
#include <algorithm>
#include <cstring>

namespace openCREST {

using protocol::ProtocolCodec;
using protocol::CommandId;
using protocol::StatusPayload;
using protocol::DATA_PACKET_BYTES;
using protocol::DATA_SAMPLES_PER_PKT;

// ---------------------------------------------------------------------------
// Test setup
// ---------------------------------------------------------------------------

void MockTransport::set_calibration(const CalibrationData& cal) {
    calibration_ = cal;
}

void MockTransport::set_initial_state(ModemState state) {
    state_ = state;
}

void MockTransport::set_settling_time_ms(uint64_t ms) {
    settling_time_ms_ = ms;
}

void MockTransport::enqueue_tx_packet(const DataPacket& pkt) {
    tx_queue_.push_back(pkt);
}

void MockTransport::enqueue_tx_waveform(const std::vector<uint16_t>& samples) {
    size_t offset = 0;
    uint16_t pkt_id = 0;
    while (offset < samples.size()) {
        const size_t n = std::min(samples.size() - offset, DATA_SAMPLES_PER_PKT);
        DataPacket pkt{};
        ProtocolCodec::encode_data_packet(pkt.data(), pkt_id, samples.data() + offset, n);
        tx_queue_.push_back(pkt);
        offset += n;
        ++pkt_id;
    }
}

void MockTransport::schedule_state_transition(uint64_t delay_ms, ModemState next_state) {
    transitions_.push_back({sim_time_ms_ + delay_ms, next_state});
    // Keep sorted ascending by time
    std::sort(transitions_.begin(), transitions_.end(),
              [](const Transition& a, const Transition& b) {
                  return a.time_ms < b.time_ms;
              });
}

// ---------------------------------------------------------------------------
// Verification
// ---------------------------------------------------------------------------

const std::vector<MockTransport::DataPacket>& MockTransport::received_rx_packets() const {
    return rx_packets_;
}

uint64_t MockTransport::simulated_time_ms() const {
    return sim_time_ms_;
}

ModemState MockTransport::current_state() const {
    return state_;
}

// ---------------------------------------------------------------------------
// IModemTransport
// ---------------------------------------------------------------------------

bool MockTransport::open(const std::string& /*device_identifier*/) {
    is_open_ = true;
    return true;
}

void MockTransport::close() {
    is_open_ = false;
}

bool MockTransport::is_open() const {
    return is_open_;
}

TransferResult MockTransport::send_data(const uint8_t* data, size_t len, int /*timeout_ms*/) {
    if (!is_open_) {
        return {TransferResult::Status::DISCONNECTED, 0};
    }
    DataPacket pkt{};
    const size_t n = std::min(len, DATA_PACKET_BYTES);
    std::memcpy(pkt.data(), data, n);
    rx_packets_.push_back(pkt);
    ++rx_expected_id_;  // wraps at 65535 → 0; mirrors real firmware counter
    return {TransferResult::Status::OK, static_cast<int>(n)};
}

TransferResult MockTransport::recv_data(uint8_t* data, size_t len, int /*timeout_ms*/) {
    if (!is_open_) {
        return {TransferResult::Status::DISCONNECTED, 0};
    }
    if (state_ != ModemState::TX || tx_queue_.empty()) {
        return {TransferResult::Status::TIMEOUT, 0};
    }

    const DataPacket& pkt = tx_queue_.front();
    const size_t n = std::min(len, DATA_PACKET_BYTES);
    std::memcpy(data, pkt.data(), n);
    tx_queue_.pop_front();

    // Auto-transition: when TX waveform is exhausted, go to SETTLING
    if (tx_queue_.empty() && state_ == ModemState::TX) {
        state_ = ModemState::SETTLING;
        // Schedule automatic RX transition after settling time
        transitions_.push_back({sim_time_ms_ + settling_time_ms_, ModemState::RX});
        std::sort(transitions_.begin(), transitions_.end(),
                  [](const Transition& a, const Transition& b) {
                      return a.time_ms < b.time_ms;
                  });
    }

    return {TransferResult::Status::OK, static_cast<int>(n)};
}

TransferResult MockTransport::send_control(const uint8_t* data, size_t len, int /*timeout_ms*/) {
    if (!is_open_) {
        return {TransferResult::Status::DISCONNECTED, 0};
    }
    if (len < 1) {
        return {TransferResult::Status::ERROR, 0};
    }

    const auto cmd = static_cast<CommandId>(data[0]);
    switch (cmd) {
    case CommandId::REQUEST_CALIBRATION:
        pending_calibration_ = true;
        break;

    case CommandId::ENTER_HIL_MODE:
        if (state_ == ModemState::IDLE) {
            state_ = ModemState::RX;
        }
        break;

    case CommandId::EXIT_HIL_MODE:
        state_ = ModemState::IDLE;
        break;

    case CommandId::SELECT_ATTENUATION:
        if (len >= 2) {
            attenuation_idx_ = data[1];
        }
        break;
    }

    return {TransferResult::Status::OK, static_cast<int>(len)};
}

TransferResult MockTransport::recv_control(uint8_t* data, size_t len, int timeout_ms) {
    if (!is_open_) {
        return {TransferResult::Status::DISCONNECTED, 0};
    }

    // Advance simulated clock
    sim_time_ms_ += static_cast<uint64_t>(timeout_ms > 0 ? timeout_ms : 1);
    apply_transitions();

    const size_t n = std::min(len, protocol::CONTROL_PACKET_BYTES);

    if (pending_calibration_) {
        pending_calibration_ = false;
        ProtocolCodec::encode_calibration(data, calibration_);
        return {TransferResult::Status::OK, static_cast<int>(n)};
    }

    // Return status packet
    StatusPayload status{};
    status.type                  = static_cast<uint8_t>(protocol::ControlType::STATUS);
    status.modem_state           = static_cast<uint8_t>(state_);
    status.buffer_fill           = 0;
    status.buffer_capacity       = 1024;
    status.attenuation_idx       = attenuation_idx_;
    status.error_flags           = static_cast<uint8_t>(error_flags_);
    status.firmware_timestamp_ms = static_cast<uint32_t>(sim_time_ms_ & 0xFFFFFFFFu);
    status.rx_expected_id        = rx_expected_id_;
    // In the mock, the fill measurement is always synchronous with rx_expected_id,
    // so the most recently received packet (rx_expected_id - 1) is the reference.
    status.fill_reference_id     = static_cast<uint16_t>(rx_expected_id_ - 1u);

    ProtocolCodec::encode_status(data, status);
    return {TransferResult::Status::OK, static_cast<int>(n)};
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void MockTransport::apply_transitions() {
    // Fire all transitions whose scheduled time has been reached
    while (!transitions_.empty() && transitions_.front().time_ms <= sim_time_ms_) {
        state_ = transitions_.front().state;
        transitions_.erase(transitions_.begin());
    }
}

} // namespace openCREST
