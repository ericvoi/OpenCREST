#include <gtest/gtest.h>
#include <array>
#include <cstddef>
#include <cstring>
#include <limits>
#include "protocol/protocol_codec.hpp"
#include "protocol/packets.hpp"
#include "transport/mock_transport.hpp"
#include "core/types.hpp"

using namespace openCREST;
using namespace openCREST::protocol;

// ---------------------------------------------------------------------------
// Data packet encode / decode
// ---------------------------------------------------------------------------

TEST(ProtocolCodec, DataPacketRoundTrip) {
    std::array<uint8_t, DATA_PACKET_BYTES> buf{};
    std::array<uint16_t, DATA_SAMPLES_PER_PKT> samples{};
    for (size_t i = 0; i < DATA_SAMPLES_PER_PKT; ++i) {
        samples[i] = static_cast<uint16_t>(i * 3 + 7);
    }

    ProtocolCodec::encode_data_packet(buf.data(), 0x1234, samples.data(), DATA_SAMPLES_PER_PKT);

    uint16_t got_id = 0;
    std::array<uint16_t, DATA_SAMPLES_PER_PKT> got_samples{};
    size_t actual = 0;
    ASSERT_TRUE(ProtocolCodec::decode_data_packet(buf.data(), got_id,
                                                   got_samples.data(),
                                                   DATA_SAMPLES_PER_PKT, actual));
    EXPECT_EQ(got_id, 0x1234);
    EXPECT_EQ(actual, DATA_SAMPLES_PER_PKT);
    EXPECT_EQ(got_samples, samples);
}

TEST(ProtocolCodec, DataPacketIdLittleEndian) {
    std::array<uint8_t, DATA_PACKET_BYTES> buf{};
    uint16_t dummy_samples[1] = {0};
    ProtocolCodec::encode_data_packet(buf.data(), 0xABCD, dummy_samples, 0);

    // First two bytes are packet_id LE
    EXPECT_EQ(buf[0], 0xCD);
    EXPECT_EQ(buf[1], 0xAB);
}

TEST(ProtocolCodec, DataPacketPartialSamplesZeroPads) {
    std::array<uint8_t, DATA_PACKET_BYTES> buf{};
    uint16_t samples[10];
    for (int i = 0; i < 10; ++i) samples[i] = static_cast<uint16_t>(100 + i);

    ProtocolCodec::encode_data_packet(buf.data(), 0, samples, 10);

    uint16_t got_id = 0;
    std::array<uint16_t, DATA_SAMPLES_PER_PKT> decoded{};
    size_t actual = 0;
    ProtocolCodec::decode_data_packet(buf.data(), got_id, decoded.data(), DATA_SAMPLES_PER_PKT, actual);

    EXPECT_EQ(actual, DATA_SAMPLES_PER_PKT);
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(decoded[i], static_cast<uint16_t>(100 + i)) << "at i=" << i;
    }
    for (size_t i = 10; i < DATA_SAMPLES_PER_PKT; ++i) {
        EXPECT_EQ(decoded[i], 0u) << "padding at i=" << i;
    }
}

TEST(ProtocolCodec, DataPacketSampleCountCappedAtMax) {
    std::array<uint8_t, DATA_PACKET_BYTES> buf{};
    // Fill buffer with test data
    std::array<uint16_t, DATA_SAMPLES_PER_PKT> samples{};
    for (size_t i = 0; i < DATA_SAMPLES_PER_PKT; ++i) samples[i] = 0xBEEF;
    ProtocolCodec::encode_data_packet(buf.data(), 0, samples.data(), DATA_SAMPLES_PER_PKT);

    uint16_t id;
    std::array<uint16_t, DATA_SAMPLES_PER_PKT + 10> big_buf{};
    size_t actual = 0;
    // Request more than a packet holds
    ProtocolCodec::decode_data_packet(buf.data(), id, big_buf.data(),
                                       DATA_SAMPLES_PER_PKT + 10, actual);
    EXPECT_EQ(actual, DATA_SAMPLES_PER_PKT);
}

// ---------------------------------------------------------------------------
// Sequence-number tracking
// ---------------------------------------------------------------------------

TEST(ProtocolCodec, SequenceNoGap) {
    ProtocolCodec codec;
    for (uint16_t i = 0; i < 10; ++i) {
        EXPECT_TRUE(codec.check_sequence(i)) << "at id=" << i;
    }
    EXPECT_EQ(codec.gap_count(), 0u);
}

TEST(ProtocolCodec, SequenceGapDetected) {
    ProtocolCodec codec;
    EXPECT_TRUE(codec.check_sequence(0));
    EXPECT_TRUE(codec.check_sequence(1));
    EXPECT_FALSE(codec.check_sequence(3));  // gap: 2 was skipped
    EXPECT_EQ(codec.gap_count(), 1u);
    // Next expected should now be 4
    EXPECT_TRUE(codec.check_sequence(4));
}

TEST(ProtocolCodec, SequenceWrapsAt65535) {
    ProtocolCodec codec;
    // Advance close to wrap
    codec.reset_sequence();
    // Jump to 65534 via check (causes one gap, but that's fine for this test)
    // Instead: use reset + check consecutive
    EXPECT_TRUE(codec.check_sequence(0));  // reset starts at 0
    // Simulate near-wrap: manually verify wrap logic
    // check 65534, 65535, 0 in sequence
    ProtocolCodec codec2;
    // Skip ahead by checking many IDs would be slow; instead check the math:
    // After check_sequence(65535), expected_ = 0 (wraps naturally via uint16_t + 1)
    // We can test by seeding with check of 65534
    EXPECT_TRUE(codec2.check_sequence(0));   // expected_ → 1
    // Fill up: check 65534 will cause a gap from 1 to 65534
    EXPECT_FALSE(codec2.check_sequence(65534));  // gap
    EXPECT_TRUE(codec2.check_sequence(65535));   // in sequence
    EXPECT_TRUE(codec2.check_sequence(0));        // wraps: 65535+1 == 0
}

TEST(ProtocolCodec, SequenceResetClearsGapCount) {
    ProtocolCodec codec;
    EXPECT_TRUE(codec.check_sequence(0));
    EXPECT_FALSE(codec.check_sequence(5));
    EXPECT_EQ(codec.gap_count(), 1u);
    codec.reset_sequence();
    EXPECT_EQ(codec.gap_count(), 0u);
    EXPECT_TRUE(codec.check_sequence(0));
}

// ---------------------------------------------------------------------------
// Command encoding
// ---------------------------------------------------------------------------

TEST(ProtocolCodec, EncodeCommandRequestCalibration) {
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_command(buf.data(), CommandId::REQUEST_CALIBRATION);

    EXPECT_EQ(buf[0], static_cast<uint8_t>(CommandId::REQUEST_CALIBRATION));
    // Remaining bytes zero
    for (size_t i = 1; i < CONTROL_PACKET_BYTES; ++i) {
        EXPECT_EQ(buf[i], 0u) << "byte " << i << " should be zero";
    }
}

TEST(ProtocolCodec, EncodeCommandWithPayload) {
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    uint8_t payload[2] = {0x01, 0x02};
    ProtocolCodec::encode_command(buf.data(), CommandId::SELECT_ATTENUATION, payload, 2);

    EXPECT_EQ(buf[0], static_cast<uint8_t>(CommandId::SELECT_ATTENUATION));
    EXPECT_EQ(buf[1], 0x01);
    EXPECT_EQ(buf[2], 0x02);
    EXPECT_EQ(buf[3], 0x00);
}

// ---------------------------------------------------------------------------
// Status decode
// ---------------------------------------------------------------------------

TEST(ProtocolCodec, DecodeStatusSuccess) {
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    StatusPayload out_status{};
    out_status.type             = static_cast<uint8_t>(ControlType::STATUS);
    out_status.modem_state      = static_cast<uint8_t>(ModemState::RX);
    out_status.buffer_fill      = 512;
    out_status.buffer_capacity  = 1024;
    out_status.attenuation_idx  = 1;
    out_status.error_flags      = 0;
    std::memset(out_status.reserved, 0, sizeof(out_status.reserved));
    ProtocolCodec::encode_status(buf.data(), out_status);

    StatusPayload parsed{};
    ASSERT_TRUE(ProtocolCodec::decode_status(buf.data(), parsed));
    EXPECT_EQ(parsed.type,            out_status.type);
    EXPECT_EQ(parsed.modem_state,     out_status.modem_state);
    EXPECT_EQ(parsed.buffer_fill,     512u);
    EXPECT_EQ(parsed.buffer_capacity, 1024u);
    EXPECT_EQ(parsed.attenuation_idx, 1u);
    EXPECT_EQ(parsed.error_flags,     0u);
}

TEST(ProtocolCodec, DecodeStatusWrongType) {
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    buf[0] = static_cast<uint8_t>(ControlType::CALIBRATION);

    StatusPayload parsed{};
    EXPECT_FALSE(ProtocolCodec::decode_status(buf.data(), parsed));
}

// ---------------------------------------------------------------------------
// Calibration encode / decode round-trip
// ---------------------------------------------------------------------------

TEST(ProtocolCodec, CalibrationRoundTrip) {
    CalibrationData cal{};
    cal.adc_bits                              = 16;
    cal.dac_bits                              = 16;
    cal.num_input_attenuations                = 2;
    cal.noise_floor_psd_counts_per_sqrt_hz    = 0.25f;
    cal.loopback_cal_attenuation              = 200;
    cal.loopback_gain                         = 0.5f;
    cal.adc_sampling_rate                     = 500'000;
    cal.dac_sampling_rate                     = 500'000;
    cal.input_attenuation[0]                  = -30.0f;
    cal.input_attenuation[1]                  = -60.0f;
    cal.output_attenuation                    = -20.0f;
    cal.center_freq_hz                        = 25'000.0f;
    cal.adc_vref_peak_volts                   = 1.65f;
    cal.dac_vref_peak_volts                   = 1.65f;

    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_calibration(buf.data(), cal);

    // First byte must be the CALIBRATION type tag
    EXPECT_EQ(buf[0], static_cast<uint8_t>(ControlType::CALIBRATION));

    CalibrationData decoded{};
    ASSERT_TRUE(ProtocolCodec::decode_calibration(buf.data(), decoded));

    EXPECT_EQ(decoded.adc_bits,                          cal.adc_bits);
    EXPECT_EQ(decoded.dac_bits,                          cal.dac_bits);
    EXPECT_EQ(decoded.num_input_attenuations,            cal.num_input_attenuations);
    EXPECT_FLOAT_EQ(decoded.noise_floor_psd_counts_per_sqrt_hz,
                    cal.noise_floor_psd_counts_per_sqrt_hz);
    EXPECT_EQ(decoded.loopback_cal_attenuation,          cal.loopback_cal_attenuation);
    EXPECT_FLOAT_EQ(decoded.loopback_gain,               cal.loopback_gain);
    EXPECT_EQ(decoded.adc_sampling_rate,                 cal.adc_sampling_rate);
    EXPECT_EQ(decoded.dac_sampling_rate,                 cal.dac_sampling_rate);
    EXPECT_FLOAT_EQ(decoded.input_attenuation[0],        cal.input_attenuation[0]);
    EXPECT_FLOAT_EQ(decoded.input_attenuation[1],        cal.input_attenuation[1]);
    EXPECT_FLOAT_EQ(decoded.output_attenuation,          cal.output_attenuation);
    EXPECT_FLOAT_EQ(decoded.center_freq_hz,              cal.center_freq_hz);
    EXPECT_FLOAT_EQ(decoded.adc_vref_peak_volts,         cal.adc_vref_peak_volts);
    EXPECT_FLOAT_EQ(decoded.dac_vref_peak_volts,         cal.dac_vref_peak_volts);
}

TEST(ProtocolCodec, CalibrationDefaultsRoundTripCleanly) {
    // Default-constructed CalibrationData round-trips — the validator
    // accepts project-wide defaults (Vref=1.0, fc=25kHz, PSD=0 for
    // "uncalibrated").
    CalibrationData cal{};
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_calibration(buf.data(), cal);

    CalibrationData decoded{};
    ASSERT_TRUE(ProtocolCodec::decode_calibration(buf.data(), decoded));
    EXPECT_FLOAT_EQ(decoded.adc_vref_peak_volts, 1.0f);
    EXPECT_FLOAT_EQ(decoded.dac_vref_peak_volts, 1.0f);
    EXPECT_FLOAT_EQ(decoded.center_freq_hz,      25'000.0f);
    EXPECT_FLOAT_EQ(decoded.noise_floor_psd_counts_per_sqrt_hz, 0.0f);
}

TEST(ProtocolCodec, DecodeCalibrationWrongType) {
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    buf[0] = static_cast<uint8_t>(ControlType::STATUS);

    CalibrationData cal{};
    EXPECT_FALSE(ProtocolCodec::decode_calibration(buf.data(), cal));
}

TEST(ProtocolCodec, DecodeCalibrationRejectsNegativeNoisePsd) {
    CalibrationData cal{};
    cal.noise_floor_psd_counts_per_sqrt_hz = 0.5f;
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_calibration(buf.data(), cal);

    // Stomp the PSD field on the wire with a negative value.
    const float bad_value = -1.0f;
    std::memcpy(buf.data() + offsetof(CalibrationPayload,
                                       noise_floor_psd_counts_per_sqrt_hz),
                &bad_value, sizeof(float));

    CalibrationData decoded{};
    EXPECT_FALSE(ProtocolCodec::decode_calibration(buf.data(), decoded));
}

TEST(ProtocolCodec, DecodeCalibrationRejectsZeroVref) {
    CalibrationData cal{};
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_calibration(buf.data(), cal);

    const float bad_value = 0.0f;
    std::memcpy(buf.data() + offsetof(CalibrationPayload, adc_vref_peak_volts),
                &bad_value, sizeof(float));

    CalibrationData decoded{};
    EXPECT_FALSE(ProtocolCodec::decode_calibration(buf.data(), decoded));
}

TEST(ProtocolCodec, DecodeCalibrationRejectsOutOfRangeCenterFreq) {
    CalibrationData cal{};
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_calibration(buf.data(), cal);

    const float bad_value = 2.0e6f;  // > 1 MHz upper limit
    std::memcpy(buf.data() + offsetof(CalibrationPayload, center_freq_hz),
                &bad_value, sizeof(float));

    CalibrationData decoded{};
    EXPECT_FALSE(ProtocolCodec::decode_calibration(buf.data(), decoded));
}

TEST(ProtocolCodec, DecodeCalibrationRejectsNanVref) {
    CalibrationData cal{};
    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};
    ProtocolCodec::encode_calibration(buf.data(), cal);

    const float bad_value = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(buf.data() + offsetof(CalibrationPayload, dac_vref_peak_volts),
                &bad_value, sizeof(float));

    CalibrationData decoded{};
    EXPECT_FALSE(ProtocolCodec::decode_calibration(buf.data(), decoded));
}

// ---------------------------------------------------------------------------
// MockTransport handshake smoke test
// ---------------------------------------------------------------------------

TEST(MockTransport, CalibrationHandshake) {
    CalibrationData cal{};
    cal.adc_bits          = 12;
    cal.dac_bits          = 12;
    cal.adc_sampling_rate = 500'000;
    cal.dac_sampling_rate = 500'000;
    cal.loopback_gain     = 1.0f;

    MockTransport mock;
    mock.set_calibration(cal);
    ASSERT_TRUE(mock.open("test-modem"));

    // Send REQUEST_CALIBRATION
    std::array<uint8_t, CONTROL_PACKET_BYTES> cmd{};
    ProtocolCodec::encode_command(cmd.data(), CommandId::REQUEST_CALIBRATION);
    auto send_res = mock.send_control(cmd.data(), cmd.size(), 10);
    EXPECT_TRUE(send_res.ok());

    // Receive calibration response
    std::array<uint8_t, CONTROL_PACKET_BYTES> resp{};
    auto recv_res = mock.recv_control(resp.data(), resp.size(), 1);
    EXPECT_TRUE(recv_res.ok());

    CalibrationData decoded{};
    ASSERT_TRUE(ProtocolCodec::decode_calibration(resp.data(), decoded));
    EXPECT_EQ(decoded.adc_bits, 12);
    EXPECT_EQ(decoded.dac_sampling_rate, 500'000u);
}

TEST(MockTransport, StateTransitionsOnSchedule) {
    MockTransport mock;
    mock.set_initial_state(ModemState::IDLE);
    mock.open("test");

    // Schedule: IDLE → RX after 50 ms, RX → TX after 100 ms total
    mock.schedule_state_transition(50,  ModemState::RX);
    mock.schedule_state_transition(100, ModemState::TX);

    EXPECT_EQ(mock.current_state(), ModemState::IDLE);

    std::array<uint8_t, CONTROL_PACKET_BYTES> buf{};

    // Each recv_control advances sim time by timeout_ms
    mock.recv_control(buf.data(), buf.size(), 30);  // t=31 → still IDLE
    EXPECT_EQ(mock.current_state(), ModemState::IDLE);

    mock.recv_control(buf.data(), buf.size(), 30);  // t=62 → RX fired
    EXPECT_EQ(mock.current_state(), ModemState::RX);

    mock.recv_control(buf.data(), buf.size(), 50);  // t=113 → TX fired
    EXPECT_EQ(mock.current_state(), ModemState::TX);
}

TEST(MockTransport, TxWaveformDeliveryAndAutoSettling) {
    MockTransport mock;
    mock.set_initial_state(ModemState::TX);
    mock.set_settling_time_ms(10);
    mock.open("test");

    // Enqueue a small TX waveform: 10 samples → 1 packet
    mock.enqueue_tx_waveform({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

    std::array<uint8_t, DATA_PACKET_BYTES> pkt{};
    auto res = mock.recv_data(pkt.data(), pkt.size(), 1);
    EXPECT_TRUE(res.ok());
    EXPECT_EQ(res.bytes_transferred, static_cast<int>(DATA_PACKET_BYTES));

    // TX queue empty → auto-transitioned to SETTLING
    EXPECT_EQ(mock.current_state(), ModemState::SETTLING);

    // Next recv_data in SETTLING state → TIMEOUT
    res = mock.recv_data(pkt.data(), pkt.size(), 1);
    EXPECT_TRUE(res.timed_out());

    // After simulated time passes settling period, transitions to RX
    std::array<uint8_t, CONTROL_PACKET_BYTES> ctrl{};
    mock.recv_control(ctrl.data(), ctrl.size(), 20);  // advances past settling_time_ms=10
    EXPECT_EQ(mock.current_state(), ModemState::RX);
}

TEST(MockTransport, RxPacketsCaptured) {
    MockTransport mock;
    mock.open("test");

    std::array<uint8_t, DATA_PACKET_BYTES> pkt{};
    pkt[0] = 0xAA;
    pkt[1] = 0xBB;

    mock.send_data(pkt.data(), pkt.size(), 1);
    mock.send_data(pkt.data(), pkt.size(), 1);

    EXPECT_EQ(mock.received_rx_packets().size(), 2u);
    EXPECT_EQ(mock.received_rx_packets()[0][0], 0xAA);
    EXPECT_EQ(mock.received_rx_packets()[1][1], 0xBB);
}
