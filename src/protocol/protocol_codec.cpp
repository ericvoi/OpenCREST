#include "protocol/protocol_codec.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <spdlog/spdlog.h>

namespace openCREST::protocol {

namespace {

// Range-check physically-meaningful calibration fields. Returns true if
// acceptable; logs a specific error otherwise. Tolerant of default-
// constructed CalibrationData so fixtures round-tripping defaults pass.
bool validate_calibration_ranges(const CalibrationData& cal) {
    if (!std::isfinite(cal.noise_floor_psd_counts_per_sqrt_hz) ||
        cal.noise_floor_psd_counts_per_sqrt_hz < 0.0f) {
        spdlog::error("calibration: noise_floor_psd_counts_per_sqrt_hz = {} is "
                      "not a finite non-negative value",
                      cal.noise_floor_psd_counts_per_sqrt_hz);
        return false;
    }
    if (!std::isfinite(cal.center_freq_hz) ||
        cal.center_freq_hz <= 0.0f ||
        cal.center_freq_hz > 1.0e6f) {
        spdlog::error("calibration: center_freq_hz = {} outside (0, 1e6] Hz",
                      cal.center_freq_hz);
        return false;
    }
    if (!std::isfinite(cal.adc_vref_peak_volts) ||
        cal.adc_vref_peak_volts <= 0.0f ||
        cal.adc_vref_peak_volts > 50.0f) {
        spdlog::error("calibration: adc_vref_peak_volts = {} outside (0, 50] V",
                      cal.adc_vref_peak_volts);
        return false;
    }
    if (!std::isfinite(cal.dac_vref_peak_volts) ||
        cal.dac_vref_peak_volts <= 0.0f ||
        cal.dac_vref_peak_volts > 50.0f) {
        spdlog::error("calibration: dac_vref_peak_volts = {} outside (0, 50] V",
                      cal.dac_vref_peak_volts);
        return false;
    }
    return true;
}

} // namespace

void ProtocolCodec::encode_data_packet(uint8_t* buf_512,
                                        uint16_t packet_id,
                                        const uint16_t* samples,
                                        size_t sample_count) {
    buf_512[0] = static_cast<uint8_t>(packet_id & 0xFF);
    buf_512[1] = static_cast<uint8_t>((packet_id >> 8) & 0xFF);

    uint16_t* dst = reinterpret_cast<uint16_t*>(buf_512 + DATA_HEADER_BYTES);
    const size_t n = std::min(sample_count, DATA_SAMPLES_PER_PKT);

    // uint16_t LE samples — plain copy on x86/ARM.
    std::memcpy(dst, samples, n * sizeof(uint16_t));

    if (n < DATA_SAMPLES_PER_PKT) {
        std::memset(dst + n, 0, (DATA_SAMPLES_PER_PKT - n) * sizeof(uint16_t));
    }
}

bool ProtocolCodec::decode_data_packet(const uint8_t* buf_512,
                                        uint16_t& packet_id,
                                        uint16_t* samples,
                                        size_t max_samples,
                                        size_t& actual_samples) {
    packet_id = static_cast<uint16_t>(buf_512[0])
              | (static_cast<uint16_t>(buf_512[1]) << 8);

    actual_samples = std::min(max_samples, DATA_SAMPLES_PER_PKT);
    const uint16_t* src = reinterpret_cast<const uint16_t*>(buf_512 + DATA_HEADER_BYTES);
    std::memcpy(samples, src, actual_samples * sizeof(uint16_t));
    return true;
}

void ProtocolCodec::encode_command(uint8_t* buf_64,
                                    CommandId cmd,
                                    const uint8_t* payload,
                                    size_t payload_len) {
    std::memset(buf_64, 0, CONTROL_PACKET_BYTES);
    buf_64[0] = static_cast<uint8_t>(cmd);
    if (payload && payload_len > 0) {
        const size_t n = std::min(payload_len, CONTROL_PACKET_BYTES - 1u);
        std::memcpy(buf_64 + 1, payload, n);
    }
}

bool ProtocolCodec::decode_status(const uint8_t* buf_64, StatusPayload& status) {
    if (buf_64[0] != static_cast<uint8_t>(ControlType::STATUS)) {
        return false;
    }
    std::memcpy(&status, buf_64, sizeof(StatusPayload));
    return true;
}

bool ProtocolCodec::decode_calibration(const uint8_t* buf_64, CalibrationData& cal) {
    if (buf_64[0] != static_cast<uint8_t>(ControlType::CALIBRATION)) {
        return false;
    }
    CalibrationPayload pkt{};
    std::memcpy(&pkt, buf_64, sizeof(CalibrationPayload));

    cal.adc_bits                              = pkt.adc_bits;
    cal.dac_bits                              = pkt.dac_bits;
    cal.num_input_attenuations                = pkt.num_input_attenuations;
    cal.loopback_cal_attenuation              = pkt.loopback_cal_attenuation;
    cal.noise_floor_psd_counts_per_sqrt_hz    = pkt.noise_floor_psd_counts_per_sqrt_hz;
    cal.loopback_gain                         = pkt.loopback_gain;
    cal.adc_sampling_rate                     = pkt.adc_sampling_rate;
    cal.dac_sampling_rate                     = pkt.dac_sampling_rate;
    cal.input_attenuation[0]                  = pkt.input_attenuation[0];
    cal.input_attenuation[1]                  = pkt.input_attenuation[1];
    cal.output_attenuation                    = pkt.output_attenuation;
    cal.center_freq_hz                        = pkt.center_freq_hz;
    cal.adc_vref_peak_volts                   = pkt.adc_vref_peak_volts;
    cal.dac_vref_peak_volts                   = pkt.dac_vref_peak_volts;

    return validate_calibration_ranges(cal);
}

void ProtocolCodec::encode_calibration(uint8_t* buf_64, const CalibrationData& cal) {
    CalibrationPayload pkt{};
    pkt.type                                = static_cast<uint8_t>(ControlType::CALIBRATION);
    pkt.adc_bits                            = cal.adc_bits;
    pkt.dac_bits                            = cal.dac_bits;
    pkt.num_input_attenuations              = cal.num_input_attenuations;
    pkt.loopback_cal_attenuation            = cal.loopback_cal_attenuation;
    pkt._reserved0                          = 0;
    pkt.noise_floor_psd_counts_per_sqrt_hz  = cal.noise_floor_psd_counts_per_sqrt_hz;
    pkt.loopback_gain                       = cal.loopback_gain;
    pkt.adc_sampling_rate                   = cal.adc_sampling_rate;
    pkt.dac_sampling_rate                   = cal.dac_sampling_rate;
    pkt.input_attenuation[0]                = cal.input_attenuation[0];
    pkt.input_attenuation[1]                = cal.input_attenuation[1];
    pkt.output_attenuation                  = cal.output_attenuation;
    pkt.center_freq_hz                      = cal.center_freq_hz;
    pkt.adc_vref_peak_volts                 = cal.adc_vref_peak_volts;
    pkt.dac_vref_peak_volts                 = cal.dac_vref_peak_volts;
    std::memset(pkt.reserved, 0, sizeof(pkt.reserved));
    std::memcpy(buf_64, &pkt, sizeof(CalibrationPayload));
}

void ProtocolCodec::encode_status(uint8_t* buf_64, const StatusPayload& status) {
    std::memcpy(buf_64, &status, sizeof(StatusPayload));
}

bool ProtocolCodec::check_sequence(uint16_t received_id) {
    if (received_id != expected_id_) {
        ++gap_count_;
        // Re-sync from the packet just received.
        expected_id_ = static_cast<uint16_t>(received_id + 1u);
        return false;
    }
    expected_id_ = static_cast<uint16_t>(received_id + 1u);  // wraps at 65535 → 0
    return true;
}

void ProtocolCodec::reset_sequence() {
    expected_id_ = 0;
    gap_count_   = 0;
}

} // namespace openCREST::protocol
