#include "modem/modem.hpp"
#include "protocol/protocol_codec.hpp"
#include "protocol/packets.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <array>
#include <cstring>

namespace openCREST {

using protocol::ProtocolCodec;
using protocol::CommandId;
using protocol::StatusPayload;

// Control-transfer timeout (ms). Tolerates one USB polling interval while
// staying short enough that the main thread isn't visibly blocked.
static constexpr int CTRL_TIMEOUT_MS = 5000;

Modem::Modem(std::unique_ptr<IModemTransport> transport,
              const std::string& modem_id)
    : transport_(std::move(transport))
    , id_(modem_id)
{}

Modem::~Modem() {
    if (transport_ && transport_->is_open()) {
        try { exit_hil_mode(); } catch (...) {}
        transport_->close();
    }
}

bool Modem::connect() {
    return transport_->open(id_);
}

CalibrationData Modem::calibrate() {
    std::array<uint8_t, protocol::CONTROL_PACKET_BYTES> cmd_buf{};
    ProtocolCodec::encode_command(cmd_buf.data(), CommandId::REQUEST_CALIBRATION);

    const auto send_result = transport_->send_control(cmd_buf.data(), cmd_buf.size(),
                                                       CTRL_TIMEOUT_MS);
    if (!send_result.ok()) {
        throw std::runtime_error("Modem::calibrate: failed to send REQUEST_CALIBRATION to '" + id_ + "'");
    }

    // The modem may send a status packet before the calibration response;
    // skip non-calibration packets up to MAX_RETRIES.
    std::array<uint8_t, protocol::CONTROL_PACKET_BYTES> resp_buf{};
    constexpr int MAX_RETRIES = 10;

    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        const auto recv_result = transport_->recv_control(resp_buf.data(), resp_buf.size(),
                                                           CTRL_TIMEOUT_MS);
        if (recv_result.timed_out()) {
            throw std::runtime_error("Modem::calibrate: timeout waiting for calibration from '" + id_ + "'");
        }
        if (!recv_result.ok()) {
            throw std::runtime_error("Modem::calibrate: transport error from '" + id_ + "'");
        }

        if (ProtocolCodec::decode_calibration(resp_buf.data(), calibration_)) {
            spdlog::info("Modem '{}': calibrated — ADC {}b @ {} SPS, DAC {}b @ {} SPS",
                         id_,
                         calibration_.adc_bits, calibration_.adc_sampling_rate,
                         calibration_.dac_bits, calibration_.dac_sampling_rate);
            return calibration_;
        }
    }

    throw std::runtime_error("Modem::calibrate: no calibration response from '" + id_ + "' after retries");
}

void Modem::enter_hil_mode() {
    std::array<uint8_t, protocol::CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_command(buf.data(), CommandId::ENTER_HIL_MODE);
    const auto result = transport_->send_control(buf.data(), buf.size(), CTRL_TIMEOUT_MS);
    if (!result.ok()) {
        throw std::runtime_error("Modem::enter_hil_mode: failed for '" + id_ + "'");
    }
    spdlog::info("Modem '{}': HIL mode entered", id_);
}

void Modem::exit_hil_mode() {
    std::array<uint8_t, protocol::CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_command(buf.data(), CommandId::EXIT_HIL_MODE);
    const auto result = transport_->send_control(buf.data(), buf.size(), CTRL_TIMEOUT_MS);
    if (!result.ok()) {
        spdlog::warn("Modem::exit_hil_mode: send failed for '{}'", id_);
    }
    spdlog::info("Modem '{}': HIL mode exited", id_);
}

void Modem::select_attenuation(uint8_t index) {
    std::array<uint8_t, protocol::CONTROL_PACKET_BYTES> buf{};
    protocol::SelectAttenuationPayload payload{index};
    ProtocolCodec::encode_command(buf.data(), CommandId::SELECT_ATTENUATION,
                                   reinterpret_cast<const uint8_t*>(&payload),
                                   sizeof(payload));
    const auto result = transport_->send_control(buf.data(), buf.size(), CTRL_TIMEOUT_MS);
    if (!result.ok()) {
        spdlog::warn("Modem '{}': select_attenuation({}) failed", id_, index);
    }
}

} // namespace openCREST
