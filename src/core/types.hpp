#pragma once
#include <atomic>
#include <cstdint>
#include "core/constants.hpp"

namespace openCREST {

enum class ModemState : uint8_t {
    IDLE     = 0,
    RX       = 1,
    TX       = 2,
    SETTLING = 3
};

struct CalibrationData {
    uint8_t  adc_bits            = 16;
    uint8_t  dac_bits            = 16;
    uint8_t  num_input_attenuations = 2;
    uint16_t loopback_cal_attenuation = 0;

    // AFE noise floor at center_freq_hz, in ADC counts/√Hz.
    // 0 means "uncalibrated"; the simulator falls back to a conservative
    // assumed AFE PSD and emits a warning.
    float    noise_floor_psd_counts_per_sqrt_hz = 0.0f;

    float    loopback_gain       = 1.0f;
    uint32_t adc_sampling_rate   = 500'000;
    uint32_t dac_sampling_rate   = 500'000;
    float    input_attenuation[2] = {0.0f, 0.0f}; // dB
    float    output_attenuation  = 0.0f;           // dB

    // Modem operating centre frequency. Used both as path-loss frequency
    // and as the noise-PSD reference frequency.
    float    center_freq_hz      = 25'000.0f;

    // Voltage full-scale at the ADC/DAC pin: a normalised sample of 1.0
    // corresponds to this many volts above mid-rail (single-ended, peak).
    // Defaults of 1.0 V make the physical-gain chain contribute 0 dB of
    // voltage anchoring (identity).
    float    adc_vref_peak_volts = 1.0f;
    float    dac_vref_peak_volts = 1.0f;

    // Audio samples packed into one data-packet payload.
    uint16_t samples_per_packet() const {
        const uint8_t bytes_per_sample = (adc_bits <= 16) ? 2 : 4;
        return static_cast<uint16_t>(
            (DATA_PACKET_SIZE - PACKET_HEADER_SIZE) / bytes_per_sample);
    }
};

struct MultipathTap {
    float delay_s     = 0.0f;   // 0 to MAX_MULTIPATH_DELAY_S
    float gain_linear = 1.0f;   // per-path amplitude (includes spreading/absorption)
    float phase_rad   = 0.0f;   // 0 for real-valued taps
};

// Shared state between the I/O thread (writer) and processing thread
// (reader). All fields are atomic; no mutex required.
struct ModemRuntimeState {
    std::atomic<ModemState> state{ModemState::IDLE};
    std::atomic<float>      buffer_fill_fraction{0.0f};
    std::atomic<uint64_t>   tx_start_ns{0};
    std::atomic<uint64_t>   state_change_ns{0};
    std::atomic<uint32_t>   error_flags{0};  // bit 0: underrun, bit 1: overrun

    ModemRuntimeState() = default;
    ModemRuntimeState(const ModemRuntimeState&) = delete;
    ModemRuntimeState& operator=(const ModemRuntimeState&) = delete;
};

} // namespace openCREST
