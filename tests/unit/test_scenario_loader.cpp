#include <gtest/gtest.h>
#include "config/scenario_loader.hpp"
#include "core/constants.hpp"
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

using namespace openCREST;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Minimal valid scenario for use as a starting point
static const char* kMinimalYaml = R"yaml(
name: test_scenario
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: modem_a
    usb_serial: "SN-001"
    transducer_id: _T
channels:
  - from: modem_a
    to: modem_a
    range_m: 150.0
)yaml";

// ---------------------------------------------------------------------------
// Basic load
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, LoadMinimalScenario) {
    const auto cfg = ScenarioLoader::load_from_string(kMinimalYaml);

    EXPECT_EQ(cfg.name, "test_scenario");
    ASSERT_EQ(cfg.modems.size(), 1u);
    EXPECT_EQ(cfg.modems[0].id, "modem_a");
    EXPECT_EQ(cfg.modems[0].usb_serial, "SN-001");
    ASSERT_EQ(cfg.channels.size(), 1u);
    EXPECT_EQ(cfg.channels[0].from_modem, "modem_a");
    EXPECT_EQ(cfg.channels[0].to_modem,   "modem_a");
    EXPECT_FLOAT_EQ(cfg.channels[0].range_m, 150.0f);
}

TEST(ScenarioLoader, DefaultEnvironmentValues) {
    const auto cfg = ScenarioLoader::load_from_string(kMinimalYaml);

    EXPECT_FLOAT_EQ(cfg.environment.sound_speed_m_s, 1500.0f);
    EXPECT_TRUE(cfg.environment.saltwater);
    EXPECT_EQ(cfg.environment.spreading_model, "spherical");
    EXPECT_FLOAT_EQ(cfg.environment.spreading_factor, 2.0f);
}

TEST(ScenarioLoader, EnvironmentOverride) {
    const char* yaml = R"yaml(
name: env_test
environment:
  sound_speed_m_s: 1490.0
  saltwater: false
  spreading_model: cylindrical
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: m
    usb_serial: SN-1
    transducer_id: _T
channels:
  - from: m
    to: m
    range_m: 100.0
)yaml";

    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg.environment.sound_speed_m_s, 1490.0f);
    EXPECT_FALSE(cfg.environment.saltwater);
    EXPECT_EQ(cfg.environment.spreading_model, "cylindrical");
    EXPECT_FLOAT_EQ(cfg.environment.spreading_factor, 1.0f);  // cylindrical
}

TEST(ScenarioLoader, EnvironmentInheritedByChannels) {
    const char* yaml = R"yaml(
name: inherit_test
environment:
  spreading_factor: 1.5
  saltwater: false
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 200.0
)yaml";

    const auto cfg = ScenarioLoader::load_from_string(yaml);
    ASSERT_FALSE(cfg.channels.empty());
    EXPECT_FLOAT_EQ(cfg.channels[0].spreading_factor, 1.5f);
    EXPECT_FALSE(cfg.channels[0].saltwater);
}

// Legacy environment.center_freq_khz key is rejected with a migration
// message — the scenario writer is otherwise silently ignored.
TEST(ScenarioLoader, LegacyEnvironmentCenterFreqKeyRejected) {
    const char* yaml = R"yaml(
name: legacy_fc
environment:
  center_freq_khz: 25.0
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Modem fields
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, ModemOptionalFields) {
    const char* yaml = R"yaml(
name: modem_test
transducers:
  T-001: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: m1
    usb_serial: SN-A
    transducer_id: T-001
    clock_offset_ppm: 3.5
    velocity_radial_m_s: 0.5
    acceleration_radial_m_s2: 0.1
channels:
  - from: m1
    to: m1
    range_m: 100.0
)yaml";

    const auto cfg = ScenarioLoader::load_from_string(yaml);
    ASSERT_FALSE(cfg.modems.empty());
    const auto& m = cfg.modems[0];
    EXPECT_EQ(m.transducer_id, "T-001");
    EXPECT_FLOAT_EQ(m.clock_offset_ppm, 3.5f);
    EXPECT_FLOAT_EQ(m.velocity_radial_m_s, 0.5f);
    EXPECT_FLOAT_EQ(m.acceleration_radial_m_s2, 0.1f);
}

// ---------------------------------------------------------------------------
// Multipath taps
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, MultipathTapsGainConversion) {
    // 20 dB → linear amplitude = 10.0; -6 dB → ~0.5; 0 dB → 1.0
    const char* yaml = R"yaml(
name: taps_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 150.0
    multipath_taps:
      - delay_s: 0.0
        gain_db: 0.0
        phase_deg: 0.0
      - delay_s: 0.01
        gain_db: -6.0206
        phase_deg: 90.0
      - delay_s: 0.02
        gain_db: 20.0
        phase_deg: 180.0
)yaml";

    const auto cfg = ScenarioLoader::load_from_string(yaml);
    ASSERT_EQ(cfg.channels[0].multipath_taps.size(), 3u);

    const auto& t0 = cfg.channels[0].multipath_taps[0];
    EXPECT_FLOAT_EQ(t0.delay_s, 0.0f);
    EXPECT_NEAR(t0.gain_linear, 1.0f, 1e-4f);
    EXPECT_FLOAT_EQ(t0.phase_rad, 0.0f);

    const auto& t1 = cfg.channels[0].multipath_taps[1];
    EXPECT_NEAR(t1.gain_linear, 0.5f, 1e-3f);  // -6.0206 dB ≈ 0.5
    EXPECT_NEAR(t1.phase_rad, static_cast<float>(M_PI / 2.0), 1e-4f);

    const auto& t2 = cfg.channels[0].multipath_taps[2];
    EXPECT_NEAR(t2.gain_linear, 10.0f, 1e-3f);  // 20 dB → 10.0
    EXPECT_NEAR(t2.phase_rad, static_cast<float>(M_PI), 1e-4f);
}

TEST(ScenarioLoader, NoTapsAddsDirectPathDefault) {
    const auto cfg = ScenarioLoader::load_from_string(kMinimalYaml);
    // No multipath_taps specified → loader adds one direct path tap
    ASSERT_EQ(cfg.channels[0].multipath_taps.size(), 1u);
    EXPECT_FLOAT_EQ(cfg.channels[0].multipath_taps[0].delay_s, 0.0f);
    EXPECT_FLOAT_EQ(cfg.channels[0].multipath_taps[0].gain_linear, 1.0f);
}

// ---------------------------------------------------------------------------
// Channel-level gain_db / direct_los / propagation_delay_s
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, ChannelDefaultsForNewFields) {
    const auto cfg = ScenarioLoader::load_from_string(kMinimalYaml);
    EXPECT_FLOAT_EQ(cfg.channels[0].gain_db, 0.0f);
    EXPECT_TRUE(cfg.channels[0].direct_los);
    EXPECT_LT(cfg.channels[0].propagation_delay_s, 0.0f);  // sentinel: derive from range
}

TEST(ScenarioLoader, ChannelGainDbParsed) {
    const char* yaml = R"yaml(
name: gain_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 150.0
    gain_db: 44.8
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg.channels[0].gain_db, 44.8f);
}

TEST(ScenarioLoader, PropagationDelayOverrideParsed) {
    const char* yaml = R"yaml(
name: refraction_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 1000.0
    propagation_delay_s: 0.75   # refracting ray > geometric
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg.channels[0].propagation_delay_s, 0.75f);
}

TEST(ScenarioLoader, NegativePropagationDelayThrows) {
    const char* yaml = R"yaml(
name: bad_delay
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
    propagation_delay_s: -0.1
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, DirectLosFalseWithReflectedTaps) {
    const char* yaml = R"yaml(
name: shadow_zone
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 500.0
    direct_los: false
    multipath_taps:
      - delay_s: 0.005
        gain_db: -3.0
      - delay_s: 0.012
        gain_db: -8.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FALSE(cfg.channels[0].direct_los);
    ASSERT_EQ(cfg.channels[0].multipath_taps.size(), 2u);
    EXPECT_FLOAT_EQ(cfg.channels[0].multipath_taps[0].delay_s, 0.005f);
}

TEST(ScenarioLoader, DirectLosFalseWithZeroDelayTapThrows) {
    const char* yaml = R"yaml(
name: contradictory
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 200.0
    direct_los: false
    multipath_taps:
      - delay_s: 0.0
        gain_db: 0.0
      - delay_s: 0.005
        gain_db: -3.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, DirectLosFalseWithoutTapsThrows) {
    const char* yaml = R"yaml(
name: no_arrivals
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 200.0
    direct_los: false
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Noise section
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, NoiseSection) {
    const char* yaml = R"yaml(
name: noise_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
noise:
  wenz_sea_state: 5
  min_margin_above_afe_db: 6.0
  saltwater: true
  tonal_sources:
    - frequency_hz: 60.0
      amplitude_linear: 0.01
      bandwidth_hz: 0.5
)yaml";

    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_EQ(cfg.noise.wenz_sea_state, 5);
    EXPECT_FLOAT_EQ(cfg.noise.min_margin_above_afe_db, 6.0f);
    EXPECT_FALSE(cfg.noise.disable);
    ASSERT_EQ(cfg.noise.tonal_sources.size(), 1u);
    EXPECT_FLOAT_EQ(cfg.noise.tonal_sources[0].frequency_hz, 60.0f);
    EXPECT_FLOAT_EQ(cfg.noise.tonal_sources[0].amplitude_linear, 0.01f);
    EXPECT_FLOAT_EQ(cfg.noise.tonal_sources[0].bandwidth_hz, 0.5f);
}

TEST(ScenarioLoader, NoiseDefaultsMinMarginTo10dB) {
    const char* yaml = R"yaml(
name: defaults_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
noise:
  wenz_sea_state: 3
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg.noise.min_margin_above_afe_db, 10.0f);
    EXPECT_FALSE(cfg.noise.disable);
}

TEST(ScenarioLoader, NoiseDisableParsed) {
    const char* yaml = R"yaml(
name: silent
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
noise:
  disable: true
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_TRUE(cfg.noise.disable);
}

TEST(ScenarioLoader, LegacyLevelAboveNoiseFloorKeyRejected) {
    const char* yaml = R"yaml(
name: legacy_noise
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
noise:
  level_above_noise_floor_db: 0.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Logging section
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, LoggingSection) {
    const char* yaml = R"yaml(
name: log_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
logging:
  log_raw_tx: true
  log_raw_rx: false
  log_processed: true
  output_directory: /tmp/hil_logs
  file_format: wav
)yaml";

    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_TRUE(cfg.logging.log_raw_tx);
    EXPECT_FALSE(cfg.logging.log_raw_rx);
    EXPECT_TRUE(cfg.logging.log_processed);
    EXPECT_EQ(cfg.logging.output_directory, "/tmp/hil_logs");
    EXPECT_EQ(cfg.logging.file_format, "wav");
}

// ---------------------------------------------------------------------------
// Two-modem scenario
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, TwoModemBidirectionalChannels) {
    const char* yaml = R"yaml(
name: two_modem
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: modem_a
    usb_serial: SN-A
    transducer_id: _T
  - id: modem_b
    usb_serial: SN-B
    transducer_id: _T
channels:
  - from: modem_a
    to: modem_b
    range_m: 500.0
  - from: modem_b
    to: modem_a
    range_m: 500.0
)yaml";

    const auto cfg = ScenarioLoader::load_from_string(yaml);
    ASSERT_EQ(cfg.modems.size(), 2u);
    ASSERT_EQ(cfg.channels.size(), 2u);
    EXPECT_EQ(cfg.channels[0].from_modem, "modem_a");
    EXPECT_EQ(cfg.channels[0].to_modem,   "modem_b");
    EXPECT_EQ(cfg.channels[1].from_modem, "modem_b");
    EXPECT_EQ(cfg.channels[1].to_modem,   "modem_a");
}

// ---------------------------------------------------------------------------
// Validation errors
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, MissingNameThrows) {
    const char* yaml = R"yaml(
modems:
  - id: a
    usb_serial: S1
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, MissingModemsThrows) {
    const char* yaml = R"yaml(
name: bad
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, EmptyModemsThrows) {
    const char* yaml = R"yaml(
name: bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems: []
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, MissingChannelsThrows) {
    const char* yaml = R"yaml(
name: bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, ChannelUnknownFromModemThrows) {
    const char* yaml = R"yaml(
name: bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: modem_a
    usb_serial: SN-A
    transducer_id: _T
channels:
  - from: unknown_modem
    to: modem_a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, ChannelUnknownToModemThrows) {
    const char* yaml = R"yaml(
name: bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: modem_a
    usb_serial: SN-A
    transducer_id: _T
channels:
  - from: modem_a
    to: unknown_modem
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, NegativeRangeThrows) {
    const char* yaml = R"yaml(
name: bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: -50.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, TapDelayOutOfRangeThrows) {
    const char* yaml = R"yaml(
name: bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
    multipath_taps:
      - delay_s: 0.500   # exceeds MAX_MULTIPATH_DELAY_S = 0.200
        gain_db: 0.0
        phase_deg: 0.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, DuplicateModemIdThrows) {
    const char* yaml = R"yaml(
name: bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: same
    usb_serial: SN-1
    transducer_id: _T
  - id: same
    usb_serial: SN-2
    transducer_id: _T
channels:
  - from: same
    to: same
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, InvalidYamlSyntaxThrows) {
    const char* yaml = "name: bad\nmodems: [\nbroken yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Description field
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, OptionalDescriptionField) {
    const char* yaml = R"yaml(
name: with_desc
description: "A test scenario"
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_EQ(cfg.description, "A test scenario");
}

TEST(ScenarioLoader, MissingDescriptionIsEmpty) {
    const auto cfg = ScenarioLoader::load_from_string(kMinimalYaml);
    EXPECT_TRUE(cfg.description.empty());
}

// ---------------------------------------------------------------------------
// max_message_duration_s
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, MaxMessageDurationDefault) {
    const auto cfg = ScenarioLoader::load_from_string(kMinimalYaml);
    EXPECT_FLOAT_EQ(cfg.environment.max_message_duration_s, 10.0f);
}

TEST(ScenarioLoader, MaxMessageDurationParsed) {
    const char* yaml = R"yaml(
name: msg_dur_test
environment:
  max_message_duration_s: 5.5
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg.environment.max_message_duration_s, 5.5f);
}

TEST(ScenarioLoader, MaxMessageDurationZeroThrows) {
    const char* yaml = R"yaml(
name: bad_dur
environment:
  max_message_duration_s: 0.0
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, MaxMessageDurationNegativeThrows) {
    const char* yaml = R"yaml(
name: bad_dur
environment:
  max_message_duration_s: -2.0
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: S1
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Clock-offset tolerance sampling
//
// clock_offset_ppm on a modem is interpreted as σ of a zero-mean truncated
// normal (bounds ±σ). Each modem draws its own actual_clock_offset_ppm at
// load time; the per-channel clock_offset_ppm is the *difference* between
// source and receiver actuals. Loopback (src == rx) is therefore always 0.
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, ClockOffsetZeroToleranceProducesZeroActual) {
    const char* yaml = R"yaml(
name: zero_tol
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: m
    usb_serial: SN-1
    transducer_id: _T
    clock_offset_ppm: 0.0
channels:
  - from: m
    to: m
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg.modems[0].clock_offset_ppm, 0.0f);
    EXPECT_FLOAT_EQ(cfg.modems[0].actual_clock_offset_ppm, 0.0f);
    EXPECT_FLOAT_EQ(cfg.channels[0].clock_offset_ppm, 0.0f);
}

TEST(ScenarioLoader, ClockOffsetActualWithinTolerance) {
    const char* yaml = R"yaml(
name: with_tol
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: m
    usb_serial: SN-1
    transducer_id: _T
    clock_offset_ppm: 5.0
channels:
  - from: m
    to: m
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg.modems[0].clock_offset_ppm, 5.0f);
    EXPECT_LE(std::fabs(cfg.modems[0].actual_clock_offset_ppm), 5.0f);
}

TEST(ScenarioLoader, ClockOffsetReproducibleUnderSameSeed) {
    const char* yaml = R"yaml(
name: seeded
random_seed: 42
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SN-A
    transducer_id: _T
    clock_offset_ppm: 10.0
  - id: b
    usb_serial: SN-B
    transducer_id: _T
    clock_offset_ppm: 10.0
channels:
  - from: a
    to: b
    range_m: 100.0
)yaml";
    const auto cfg1 = ScenarioLoader::load_from_string(yaml);
    const auto cfg2 = ScenarioLoader::load_from_string(yaml);
    EXPECT_FLOAT_EQ(cfg1.modems[0].actual_clock_offset_ppm,
                    cfg2.modems[0].actual_clock_offset_ppm);
    EXPECT_FLOAT_EQ(cfg1.modems[1].actual_clock_offset_ppm,
                    cfg2.modems[1].actual_clock_offset_ppm);
}

TEST(ScenarioLoader, ClockOffsetDifferentSeedsProduceDifferentDraws) {
    const char* tpl = R"yaml(
name: seed_var
random_seed: %d
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: m
    usb_serial: SN-1
    transducer_id: _T
    clock_offset_ppm: 10.0
channels:
  - from: m
    to: m
    range_m: 100.0
)yaml";
    char y1[256], y2[256];
    std::snprintf(y1, sizeof(y1), tpl, 1);
    std::snprintf(y2, sizeof(y2), tpl, 99999);
    const auto c1 = ScenarioLoader::load_from_string(y1);
    const auto c2 = ScenarioLoader::load_from_string(y2);
    EXPECT_NE(c1.modems[0].actual_clock_offset_ppm,
              c2.modems[0].actual_clock_offset_ppm);
}

TEST(ScenarioLoader, ClockOffsetTwoModemsDrawIndependently) {
    // Two modems with identical tolerance get distinct actual draws.
    const char* yaml = R"yaml(
name: pair
random_seed: 7
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SN-A
    transducer_id: _T
    clock_offset_ppm: 10.0
  - id: b
    usb_serial: SN-B
    transducer_id: _T
    clock_offset_ppm: 10.0
channels:
  - from: a
    to: b
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_NE(cfg.modems[0].actual_clock_offset_ppm,
              cfg.modems[1].actual_clock_offset_ppm);
}

TEST(ScenarioLoader, ChannelClockOffsetIsSourceMinusReceiver) {
    const char* yaml = R"yaml(
name: pair_diff
random_seed: 7
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SN-A
    transducer_id: _T
    clock_offset_ppm: 10.0
  - id: b
    usb_serial: SN-B
    transducer_id: _T
    clock_offset_ppm: 10.0
channels:
  - from: a
    to: b
    range_m: 100.0
  - from: b
    to: a
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    const float a = cfg.modems[0].actual_clock_offset_ppm;
    const float b = cfg.modems[1].actual_clock_offset_ppm;
    EXPECT_FLOAT_EQ(cfg.channels[0].clock_offset_ppm, a - b);
    EXPECT_FLOAT_EQ(cfg.channels[1].clock_offset_ppm, b - a);
    // Reciprocal channels carry equal-magnitude, opposite-sign offsets.
    EXPECT_FLOAT_EQ(cfg.channels[0].clock_offset_ppm,
                    -cfg.channels[1].clock_offset_ppm);
}

TEST(ScenarioLoader, LoopbackChannelClockOffsetIsZero) {
    // src == rx ⇒ effective offset = 0 regardless of tolerance.
    const char* yaml = R"yaml(
name: loop
random_seed: 1
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: m
    usb_serial: SN-1
    transducer_id: _T
    clock_offset_ppm: 50.0
channels:
  - from: m
    to: m
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_NE(cfg.modems[0].actual_clock_offset_ppm, 0.0f);
    EXPECT_FLOAT_EQ(cfg.channels[0].clock_offset_ppm, 0.0f);
}

TEST(ScenarioLoader, NegativeClockOffsetToleranceThrows) {
    const char* yaml = R"yaml(
name: bad_tol
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: m
    usb_serial: SN-1
    transducer_id: _T
    clock_offset_ppm: -1.0
channels:
  - from: m
    to: m
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Transducer parsing
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, ParsesTransducersBlockWithFlatTvrRvr) {
    const char* yaml = R"yaml(
name: tx_block
transducers:
  T-001:
    tvr_db: 152.0
    rvr_db: -180.0
  T-002:
    tvr_db: 150.0
    rvr_db: -178.0
modems:
  - id: modem_a
    usb_serial: SN-A
    transducer_id: T-001
  - id: modem_b
    usb_serial: SN-B
    transducer_id: T-002
channels:
  - from: modem_a
    to: modem_b
    range_m: 100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    ASSERT_EQ(cfg.transducers.size(), 2u);

    const auto it1 = cfg.transducers.find("T-001");
    ASSERT_NE(it1, cfg.transducers.end());
    EXPECT_FLOAT_EQ(it1->second.tvr_db,  152.0f);
    EXPECT_FLOAT_EQ(it1->second.rvr_db, -180.0f);

    const auto it2 = cfg.transducers.find("T-002");
    ASSERT_NE(it2, cfg.transducers.end());
    EXPECT_FLOAT_EQ(it2->second.tvr_db,  150.0f);
    EXPECT_FLOAT_EQ(it2->second.rvr_db, -178.0f);
}

TEST(ScenarioLoader, RejectsModemTransducerIdMissing) {
    // transducers block present, modem omits transducer_id.
    const char* yaml = R"yaml(
name: bad_modem
transducers:
  T-001:
    tvr_db: 152.0
    rvr_db: -180.0
modems:
  - id: modem_a
    usb_serial: SN-A
channels:
  - from: modem_a
    to: modem_a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, RejectsModemTransducerIdUnknown) {
    // Modem references an id not in the transducers block.
    const char* yaml = R"yaml(
name: bad_modem
transducers:
  T-001:
    tvr_db: 152.0
    rvr_db: -180.0
modems:
  - id: modem_a
    usb_serial: SN-A
    transducer_id: T-999
channels:
  - from: modem_a
    to: modem_a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, RejectsTransducerEntryMissingTvrOrRvr) {
    // T-001 lacks rvr_db.
    const char* yaml = R"yaml(
name: bad_tx
transducers:
  T-001:
    tvr_db: 152.0
modems:
  - id: modem_a
    usb_serial: SN-A
    transducer_id: T-001
channels:
  - from: modem_a
    to: modem_a
    range_m: 100.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Geometric channel mode
// ---------------------------------------------------------------------------

TEST(ScenarioLoader, StaticModeIsDefault) {
    // No explicit `mode:` field → channel is Static.
    const auto cfg = ScenarioLoader::load_from_string(kMinimalYaml);
    EXPECT_EQ(cfg.channels[0].mode, ChannelMode::Static);
}

TEST(ScenarioLoader, LoadGeometricChannel) {
    const char* yaml = R"yaml(
name: geom_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
    velocity_radial_m_s: -2.0
  - id: b
    usb_serial: SB
    transducer_id: _T
channels:
  - from: a
    to: b
    range_m: 1000.0
    mode: geometric
    initial_range_m: 1000.0
    geometry:
      water_depth_m: 120.0
      source_depth_m: 50.0
      receiver_depth_m: 100.0
      gamma_surface: -0.9
      gamma_bottom: 0.7
      spreading_exponent_k: 1.5
      enable_direct: true
      enable_surface: true
      enable_bottom: true
      enable_surface_bottom: true
      enable_bottom_surface: true
      r_min_m: 500.0
      r_max_m: 1100.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    ASSERT_EQ(cfg.channels.size(), 1u);
    const auto& cc = cfg.channels[0];
    EXPECT_EQ(cc.mode, ChannelMode::Geometric);
    EXPECT_FLOAT_EQ(cc.initial_range_m, 1000.0f);
    const auto& g = cc.geometry;
    EXPECT_FLOAT_EQ(g.water_depth_m,        120.0f);
    EXPECT_FLOAT_EQ(g.source_depth_m,        50.0f);
    EXPECT_FLOAT_EQ(g.receiver_depth_m,     100.0f);
    EXPECT_FLOAT_EQ(g.gamma_surface,         -0.9f);
    EXPECT_FLOAT_EQ(g.gamma_bottom,           0.7f);
    EXPECT_FLOAT_EQ(g.spreading_exponent_k,   1.5f);
    EXPECT_TRUE (g.enable_direct);
    EXPECT_TRUE (g.enable_surface);
    EXPECT_TRUE (g.enable_bottom);
    EXPECT_TRUE (g.enable_surface_bottom);
    EXPECT_TRUE (g.enable_bottom_surface);
    EXPECT_FLOAT_EQ(g.r_min_m, 500.0f);
    EXPECT_FLOAT_EQ(g.r_max_m, 1100.0f);
    // multipath_taps stays empty in geometric mode — taps are computed at
    // message start from the scene, not from YAML.
    EXPECT_TRUE(cc.multipath_taps.empty());
}

TEST(ScenarioLoader, GeometricChannelDefaultsRminRmax) {
    // r_min_m and r_max_m omitted → loader fills in R_0/2 and R_0*2.
    const char* yaml = R"yaml(
name: geom_defaults
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 400.0
    mode: geometric
    initial_range_m: 400.0
    geometry:
      water_depth_m: 100.0
      source_depth_m: 30.0
      receiver_depth_m: 30.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    const auto& g = cfg.channels[0].geometry;
    EXPECT_FLOAT_EQ(g.r_min_m, 200.0f);
    EXPECT_FLOAT_EQ(g.r_max_m, 800.0f);
}

TEST(ScenarioLoader, GeometricChannelFallsBackToRangeMWhenNoInitial) {
    // initial_range_m omitted → r_min/r_max default off range_m.
    const char* yaml = R"yaml(
name: geom_no_initial
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 600.0
    mode: geometric
    geometry:
      water_depth_m: 100.0
      source_depth_m: 30.0
      receiver_depth_m: 30.0
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_LT(cfg.channels[0].initial_range_m, 0.0f);  // sentinel preserved
    EXPECT_FLOAT_EQ(cfg.channels[0].geometry.r_min_m, 300.0f);
    EXPECT_FLOAT_EQ(cfg.channels[0].geometry.r_max_m, 1200.0f);
}

TEST(ScenarioLoader, GeometricChannelMissingGeometryBlockThrows) {
    const char* yaml = R"yaml(
name: geom_no_block
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 1000.0
    mode: geometric
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, GeometricChannelAcceptsMaxBounces) {
    const char* yaml = R"yaml(
name: geom_bounces
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 400.0
    mode: geometric
    geometry:
      water_depth_m: 100.0
      source_depth_m: 30.0
      receiver_depth_m: 30.0
      max_bounces: 4
)yaml";
    const auto cfg = ScenarioLoader::load_from_string(yaml);
    EXPECT_EQ(cfg.channels[0].geometry.max_bounces, 4);
}

TEST(ScenarioLoader, GeometricChannelRejectsOutOfRangeMaxBounces) {
    const char* yaml = R"yaml(
name: geom_bounces_bad
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 400.0
    mode: geometric
    geometry:
      water_depth_m: 100.0
      source_depth_m: 30.0
      receiver_depth_m: 30.0
      max_bounces: 5
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, GeometricChannelWithMultipathTapsThrows) {
    const char* yaml = R"yaml(
name: geom_collision
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 1000.0
    mode: geometric
    geometry:
      water_depth_m: 100.0
      source_depth_m: 50.0
      receiver_depth_m: 50.0
    multipath_taps:
      - delay_s: 0.0
        gain_db: 0.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, GeometricChannelInvalidModeStringThrows) {
    const char* yaml = R"yaml(
name: geom_bad_mode
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 1000.0
    mode: bogus
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, GeometricChannelInvalidDepthsThrows) {
    // source_depth >= water_depth.
    const char* yaml = R"yaml(
name: geom_bad_depths
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 1000.0
    mode: geometric
    geometry:
      water_depth_m: 100.0
      source_depth_m: 100.0
      receiver_depth_m: 50.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

TEST(ScenarioLoader, GeometricChannelInvalidRangeEnvelopeThrows) {
    // r_min_m >= initial_range_m → R_0 outside the envelope.
    const char* yaml = R"yaml(
name: geom_bad_env
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
channels:
  - from: a
    to: a
    range_m: 1000.0
    mode: geometric
    initial_range_m: 1000.0
    geometry:
      water_depth_m: 100.0
      source_depth_m: 50.0
      receiver_depth_m: 50.0
      r_min_m: 1100.0
      r_max_m: 2000.0
)yaml";
    EXPECT_THROW(ScenarioLoader::load_from_string(yaml), ScenarioLoadError);
}

// ---------------------------------------------------------------------------
// Replay mode
// ---------------------------------------------------------------------------

#include <fstream>
#include "test_helpers/octt_writer.hpp"

namespace {

// Two-tap .octt (max delay 10 ms) written into gtest's temp dir.
std::string write_replay_fixture(const std::string& name) {
    const std::string path = testing::TempDir() + name;
    std::vector<double> delays;
    std::vector<float>  amps;
    for (int f = 0; f < 3; ++f) {
        delays.insert(delays.end(), {0.001, 0.010});
        amps.insert(amps.end(),   {1.0f, 0.5f});
    }
    test_helpers::write_octt_file(path, 2, 3, 0.050, 35e3, delays, amps);
    return path;
}

std::string replay_yaml(const std::string& channel_extras,
                        const std::string& modem_extras = "") {
    return std::string(R"yaml(
name: replay_test
transducers:
  _T: {tvr_db: 0.0, rvr_db: 0.0}
modems:
  - id: a
    usb_serial: SA
    transducer_id: _T
)yaml") + modem_extras + R"yaml(
  - id: b
    usb_serial: SB
    transducer_id: _T
channels:
  - from: a
    to: b
    range_m: 800.0
    mode: replay
)yaml" + channel_extras;
}

} // namespace

TEST(ScenarioLoader, LoadReplayChannel) {
    const auto octt = write_replay_fixture("loader_ok.octt");
    const auto cfg = ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n"
        "      trajectory_file: \"" + octt + "\"\n"
        "      offset_s: 1.5\n"
        "      advance_per_message: true\n"
        "      wrap_if_remaining_lt_s: 5.0\n"));
    ASSERT_EQ(cfg.channels.size(), 1u);
    const auto& cc = cfg.channels[0];
    EXPECT_EQ(cc.mode, ChannelMode::Replay);
    EXPECT_EQ(cc.replay.trajectory_path, octt);
    EXPECT_DOUBLE_EQ(cc.replay.offset_s, 1.5);
    EXPECT_TRUE(cc.replay.advance_per_message);
    EXPECT_DOUBLE_EQ(cc.replay.wrap_if_remaining_lt_s, 5.0);
    // Sizing fields lifted from the file header.
    EXPECT_DOUBLE_EQ(cc.replay.max_delay_s, 0.010);
    EXPECT_EQ(cc.replay.tap_count, 2u);
    EXPECT_TRUE(cc.multipath_taps.empty());
}

TEST(ScenarioLoader, ReplayDefaultsAreFixedModeFromRecordStart) {
    const auto octt = write_replay_fixture("loader_defaults.octt");
    const auto cfg = ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n"
        "      trajectory_file: \"" + octt + "\"\n"));
    const auto& r = cfg.channels[0].replay;
    EXPECT_DOUBLE_EQ(r.offset_s, 0.0);
    EXPECT_FALSE(r.advance_per_message);
    EXPECT_DOUBLE_EQ(r.wrap_if_remaining_lt_s, 0.0);
}

TEST(ScenarioLoader, ReplayMissingBlockThrows) {
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml("")),
                 ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayMissingTrajectoryFileThrows) {
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n      offset_s: 0.0\n")), ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayNonexistentTrajectoryThrows) {
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n      trajectory_file: \"/nonexistent/x.octt\"\n")),
        ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayCorruptTrajectoryThrows) {
    const std::string bad = testing::TempDir() + "corrupt.octt";
    std::ofstream(bad, std::ios::binary) << "not an octt file";
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n      trajectory_file: \"" + bad + "\"\n")),
        ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayWithMultipathTapsThrows) {
    const auto octt = write_replay_fixture("loader_taps.octt");
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n      trajectory_file: \"" + octt + "\"\n"
        "    multipath_taps:\n      - delay_s: 0.0\n")),
        ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayWithGeometryThrows) {
    const auto octt = write_replay_fixture("loader_geom.octt");
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n      trajectory_file: \"" + octt + "\"\n"
        "    geometry:\n      water_depth_m: 100.0\n")),
        ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayWithInitialRangeThrows) {
    const auto octt = write_replay_fixture("loader_ir.octt");
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n      trajectory_file: \"" + octt + "\"\n"
        "    initial_range_m: 500.0\n")),
        ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayNegativeOffsetThrows) {
    const auto octt = write_replay_fixture("loader_neg.octt");
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n"
        "      trajectory_file: \"" + octt + "\"\n"
        "      offset_s: -0.1\n")),
        ScenarioLoadError);
}

TEST(ScenarioLoader, ReplaySourceModemMotionThrows) {
    const auto octt = write_replay_fixture("loader_motion.octt");
    EXPECT_THROW(ScenarioLoader::load_from_string(replay_yaml(
        "    replay:\n      trajectory_file: \"" + octt + "\"\n",
        "    velocity_radial_m_s: -2.0\n")),
        ScenarioLoadError);
}

TEST(ScenarioLoader, ReplayRelativePathResolvesAgainstScenarioDir) {
    // Scenario file and trajectory sit in the same temp directory; the
    // YAML references the trajectory by bare filename.
    const auto octt = write_replay_fixture("loader_rel.octt");
    const std::string yaml_path = testing::TempDir() + "loader_rel.yaml";
    std::ofstream(yaml_path) << replay_yaml(
        "    replay:\n      trajectory_file: \"loader_rel.octt\"\n");

    const auto cfg = ScenarioLoader::load(yaml_path);
    EXPECT_EQ(cfg.channels[0].replay.trajectory_path, octt);
}
