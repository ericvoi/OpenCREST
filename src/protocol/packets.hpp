#pragma once
#include <cstdint>
#include <cstddef>

// Wire-format packet definitions for the modem ↔ host USB protocol.
// All multi-byte fields are little-endian. Structs are packed so sizeof()
// matches the wire size; static_asserts below enforce this.
//
// This file is the contract between host software and modem firmware.

namespace openCREST::protocol {

// ---------------------------------------------------------------------------
// Packet size constants
// ---------------------------------------------------------------------------

constexpr size_t DATA_PACKET_BYTES    = 512;
constexpr size_t CONTROL_PACKET_BYTES = 64;
constexpr size_t DATA_HEADER_BYTES    = 2;       // sizeof packet_id field
constexpr size_t DATA_SAMPLES_PER_PKT = (DATA_PACKET_BYTES - DATA_HEADER_BYTES)
                                        / sizeof(uint16_t);  // = 255

// ---------------------------------------------------------------------------
// Control packet type byte (first byte of every control packet)
// ---------------------------------------------------------------------------

enum class ControlType : uint8_t {
    STATUS      = 0x01,  // modem → host: runtime status
    CALIBRATION = 0x02,  // modem → host: calibration data
};

// ---------------------------------------------------------------------------
// Command IDs (host → modem, first byte of CommandPacket)
// ---------------------------------------------------------------------------

enum class CommandId : uint8_t {
    REQUEST_CALIBRATION = 0x10,
    ENTER_HIL_MODE      = 0x11,
    EXIT_HIL_MODE       = 0x12,
    SELECT_ATTENUATION  = 0x13,
};

// ---------------------------------------------------------------------------
// Status packet (modem → host)  — 64 bytes total
// ---------------------------------------------------------------------------

struct StatusPayload {
    uint8_t  type;                     // ControlType::STATUS = 0x01
    uint8_t  modem_state;              // ModemState enum value (IDLE/RX/TX/SETTLING)
    uint16_t buffer_fill;              // Current buffer occupancy in samples
    uint16_t buffer_capacity;          // Total buffer capacity in samples
    uint8_t  attenuation_idx;          // Active input attenuation index (0 or 1)
    uint8_t  error_flags;              // bit 0: RX underrun, bit 1: RX overrun
    uint32_t firmware_timestamp_ms;    // Modem's internal millisecond counter (wraps ~49 days)
    uint16_t rx_expected_id;           // Modem's next-expected host→modem packet_id; wraps at 65535
    uint16_t fill_reference_id;        // The host→modem packet_id whose arrival triggered buffer_fill
    uint8_t  reserved[48];
} __attribute__((packed));

static_assert(sizeof(StatusPayload) == CONTROL_PACKET_BYTES,
              "StatusPayload must be exactly 64 bytes");

// ---------------------------------------------------------------------------
// Calibration response (modem → host)  — 64 bytes total
//
// Field offsets (packed, fields are kept naturally aligned):
//   [ 0]  4 × uint8                                     → 4
//   [ 4]  uint16  loopback_cal_attenuation              → 6
//   [ 6]  uint16  _reserved0 (alignment)                → 8
//   [ 8]  float   noise_floor_psd_counts_per_sqrt_hz    → 12
//   [12]  float   loopback_gain                         → 16
//   [16]  uint32  adc_sampling_rate                     → 20
//   [20]  uint32  dac_sampling_rate                     → 24
//   [24]  float   input_attenuation[2]                  → 32
//   [32]  float   output_attenuation                    → 36
//   [36]  float   center_freq_hz                        → 40
//   [40]  float   adc_vref_peak_volts                   → 44
//   [44]  float   dac_vref_peak_volts                   → 48
//   [48] 16 × uint8  reserved                           → 64
// ---------------------------------------------------------------------------

struct CalibrationPayload {
    uint8_t  type;                              // ControlType::CALIBRATION = 0x02
    uint8_t  adc_bits;
    uint8_t  dac_bits;
    uint8_t  num_input_attenuations;
    uint16_t loopback_cal_attenuation;
    uint16_t _reserved0;                        // alignment slot
    // AFE noise floor PSD at center_freq_hz, in ADC-counts/√Hz. Firmware
    // reports as rms_counts / sqrt(measurement_bw_hz). PSD form decouples
    // the host from the modem's measurement bandwidth / decimation choices.
    float    noise_floor_psd_counts_per_sqrt_hz;
    float    loopback_gain;
    uint32_t adc_sampling_rate;
    uint32_t dac_sampling_rate;
    float    input_attenuation[2];              // dB, index 0 = low-attenuation setting
    float    output_attenuation;                // dB
    float    center_freq_hz;                    // Modem operating center frequency
    float    adc_vref_peak_volts;               // ADC full-scale: sample 1.0 ↔ this many volts above mid-rail
    float    dac_vref_peak_volts;               // DAC full-scale: sample 1.0 ↔ this many volts above mid-rail
    uint8_t  reserved[16];
} __attribute__((packed));

static_assert(sizeof(CalibrationPayload) == CONTROL_PACKET_BYTES,
              "CalibrationPayload must be exactly 64 bytes");

// ---------------------------------------------------------------------------
// Command packet (host → modem)  — 64 bytes total
// ---------------------------------------------------------------------------

struct CommandPacket {
    uint8_t command_id;     // CommandId enum value
    uint8_t payload[63];    // Command-specific data, zero-padded
} __attribute__((packed));

static_assert(sizeof(CommandPacket) == CONTROL_PACKET_BYTES,
              "CommandPacket must be exactly 64 bytes");

// ---------------------------------------------------------------------------
// SELECT_ATTENUATION command payload (in CommandPacket::payload[0])
// ---------------------------------------------------------------------------

struct SelectAttenuationPayload {
    uint8_t attenuation_idx;  // 0 or 1
} __attribute__((packed));

} // namespace openCREST::protocol
