#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "core/tap_trajectory.hpp"

using openCREST::TapTrajectory;
using openCREST::TapTrajectoryError;

namespace {

// ---------------------------------------------------------------------------
// Raw .octt writer for crafting valid and deliberately corrupt files.
// Layout must mirror experiments/lib/tap_trajectory.py.
// ---------------------------------------------------------------------------

struct RawHeader {
    char     magic[4]     = {'O', 'C', 'T', 'T'};
    uint32_t version      = 1;
    uint32_t tap_count    = 0;
    uint32_t frame_count  = 0;
    double   dt_s         = 0.0;
    double   fc_meas_hz   = 0.0;
    double   max_delay_s  = 0.0;
    uint8_t  reserved[24] = {};
};
static_assert(sizeof(RawHeader) == 64);

struct RawRecord {
    double delay_s   = 0.0;
    float  amplitude = 0.0f;
    float  reserved  = 0.0f;
};
static_assert(sizeof(RawRecord) == 16);

// delays/amps are frame-major: frame_count * tap_count entries.
std::string write_octt(const RawHeader& hdr,
                       const std::vector<double>& delays,
                       const std::vector<float>& amps,
                       const std::string& name,
                       size_t truncate_bytes = 0,
                       size_t extra_bytes = 0) {
    const std::string path = testing::TempDir() + name;
    std::vector<char> bytes(sizeof(RawHeader) + delays.size() * sizeof(RawRecord)
                            + extra_bytes, 0);
    std::memcpy(bytes.data(), &hdr, sizeof(hdr));
    for (size_t i = 0; i < delays.size(); ++i) {
        RawRecord rec;
        rec.delay_s   = delays[i];
        rec.amplitude = amps[i];
        std::memcpy(bytes.data() + sizeof(RawHeader) + i * sizeof(RawRecord),
                    &rec, sizeof(rec));
    }
    if (truncate_bytes > 0) bytes.resize(bytes.size() - truncate_bytes);
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    f.close();
    return path;
}

// A minimal valid two-tap, three-frame file. Callers mutate the header or
// data to produce each rejection case.
struct ValidFile {
    RawHeader hdr;
    std::vector<double> delays;
    std::vector<float>  amps;

    ValidFile() {
        hdr.tap_count   = 2;
        hdr.frame_count = 3;
        hdr.dt_s        = 0.025;
        hdr.fc_meas_hz  = 35000.0;
        hdr.max_delay_s = 0.011;
        delays = {0.002, 0.010,
                  0.003, 0.011,
                  0.004, 0.009};
        amps   = {1.0f, 0.5f,
                  0.8f, 0.4f,
                  0.9f, 0.3f};
    }
};

const std::string kFixturePath =
    std::string(OPENCREST_TEST_DATA_DIR) + "/replay_fixture_two_tap.octt";

} // namespace

// ---------------------------------------------------------------------------
// Golden fixture (written by experiments/lib/tap_trajectory.py — the
// cross-language contract anchor).
// ---------------------------------------------------------------------------

TEST(TapTrajectory, LoadsGoldenFixtureHeader) {
    const auto traj = TapTrajectory::load(kFixturePath);
    EXPECT_EQ(traj.tap_count(), 2u);
    EXPECT_EQ(traj.frame_count(), 5u);
    EXPECT_DOUBLE_EQ(traj.dt_s(), 0.025);
    EXPECT_DOUBLE_EQ(traj.fc_meas_hz(), 35000.0);
    EXPECT_DOUBLE_EQ(traj.max_delay_s(), 0.011);
    EXPECT_DOUBLE_EQ(traj.duration_s(), 4 * 0.025);
}

TEST(TapTrajectory, PeekHeaderAgreesWithLoad) {
    const auto hdr  = TapTrajectory::peek_header(kFixturePath);
    const auto traj = TapTrajectory::load(kFixturePath);
    EXPECT_EQ(hdr.tap_count, traj.tap_count());
    EXPECT_EQ(hdr.frame_count, traj.frame_count());
    EXPECT_DOUBLE_EQ(hdr.dt_s, traj.dt_s());
    EXPECT_DOUBLE_EQ(hdr.fc_meas_hz, traj.fc_meas_hz());
    EXPECT_DOUBLE_EQ(hdr.max_delay_s, traj.max_delay_s());
}

// Expected values computed with lib/tap_trajectory.catmull_rom_uniform
// (the Python reference implementation) on the fixture tracks.
TEST(TapTrajectory, SampleMatchesPythonReference) {
    const auto traj = TapTrajectory::load(kFixturePath);

    struct Case { size_t tap; double t; double delay; float amp; };
    const Case cases[] = {
        {0, 0.0000, 0.002,                  1.0f},
        {0, 0.0125, 0.0025312500000000001,  0.893750008f},
        {0, 0.0300, 0.0029640000000000001,  0.805600007f},
        {0, 0.0625, 0.0032500000000000003,  0.812499978f},
        {0, 0.1000, 0.0035000000000000005,  0.600000024f},
        {1, 0.0000, 0.01,                   0.5f},
        {1, 0.0125, 0.01021875,             0.231249999f},
        {1, 0.0300, 0.010624,               0.0120000019f},
        {1, 0.0625, 0.010562500000000001,   0.38125001f},
        {1, 0.1000, 0.009499999999999998,   0.200000003f},
    };
    for (const auto& c : cases) {
        const auto s = traj.sample(c.tap, c.t);
        EXPECT_NEAR(s.delay_s, c.delay, 1e-12)
            << "tap " << c.tap << " t " << c.t;
        EXPECT_NEAR(s.amplitude, c.amp, 1e-6f)
            << "tap " << c.tap << " t " << c.t;
    }
}

TEST(TapTrajectory, SampleClampsTimeToRecordEndpoints) {
    const auto traj = TapTrajectory::load(kFixturePath);
    const auto before = traj.sample(0, -1.0);
    const auto at0    = traj.sample(0, 0.0);
    EXPECT_DOUBLE_EQ(before.delay_s, at0.delay_s);
    EXPECT_FLOAT_EQ(before.amplitude, at0.amplitude);

    const auto after  = traj.sample(1, 99.0);
    const auto at_end = traj.sample(1, traj.duration_s());
    EXPECT_DOUBLE_EQ(after.delay_s, at_end.delay_s);
    EXPECT_FLOAT_EQ(after.amplitude, at_end.amplitude);
}

TEST(TapTrajectory, SampleHitsFrameValuesExactly) {
    const auto traj = TapTrajectory::load(kFixturePath);
    // Frame 2 of the fixture: tap0 {0.0025, 0.9}, tap1 {0.0110, 0.3}.
    const auto s0 = traj.sample(0, 2 * 0.025);
    EXPECT_DOUBLE_EQ(s0.delay_s, 0.0025);
    EXPECT_FLOAT_EQ(s0.amplitude, 0.9f);
    const auto s1 = traj.sample(1, 2 * 0.025);
    EXPECT_DOUBLE_EQ(s1.delay_s, 0.0110);
    EXPECT_FLOAT_EQ(s1.amplitude, 0.3f);
}

// Catmull-Rom can undershoot below zero around an amplitude fade; negative
// amplitude is meaningless (phase lives in the delay), so sample() clamps.
TEST(TapTrajectory, SampleClampsAmplitudeAtZero) {
    ValidFile v;
    v.hdr.tap_count   = 1;
    v.hdr.frame_count = 4;
    v.hdr.max_delay_s = 0.001;
    v.delays = {0.001, 0.001, 0.001, 0.001};
    v.amps   = {1.0f, 0.0f, 0.0f, 1.0f};   // undershoots at mid-segment
    const auto path = write_octt(v.hdr, v.delays, v.amps, "clamp.octt");
    const auto traj = TapTrajectory::load(path);
    // Raw Catmull-Rom at t = 1.5*dt gives -0.125.
    const auto s = traj.sample(0, 1.5 * v.hdr.dt_s);
    EXPECT_FLOAT_EQ(s.amplitude, 0.0f);
    EXPECT_DOUBLE_EQ(s.delay_s, 0.001);
}

// ---------------------------------------------------------------------------
// Validation matrix — every malformed file is rejected with
// TapTrajectoryError (from both load and peek_header where applicable).
// ---------------------------------------------------------------------------

TEST(TapTrajectory, RejectsMissingFile) {
    EXPECT_THROW(TapTrajectory::load("/nonexistent/nope.octt"),
                 TapTrajectoryError);
    EXPECT_THROW(TapTrajectory::peek_header("/nonexistent/nope.octt"),
                 TapTrajectoryError);
}

TEST(TapTrajectory, RejectsBadMagic) {
    ValidFile v;
    v.hdr.magic[0] = 'X';
    const auto path = write_octt(v.hdr, v.delays, v.amps, "bad_magic.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
    EXPECT_THROW(TapTrajectory::peek_header(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsUnsupportedVersion) {
    ValidFile v;
    v.hdr.version = 2;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "bad_version.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsTapCountZero) {
    ValidFile v;
    v.hdr.tap_count = 0;
    const auto path = write_octt(v.hdr, {}, {}, "taps0.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsTapCountAboveLimit) {
    ValidFile v;
    v.hdr.tap_count = 33;   // MAX_TAPS_PER_CHANNEL + 1
    std::vector<double> delays(3 * 33, 0.001);
    std::vector<float>  amps(3 * 33, 1.0f);
    v.hdr.max_delay_s = 0.001;
    const auto path = write_octt(v.hdr, delays, amps, "taps33.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsFrameCountBelowTwo) {
    ValidFile v;
    v.hdr.frame_count = 1;
    v.delays.resize(2);
    v.amps.resize(2);
    v.hdr.max_delay_s = 0.010;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "frames1.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsNonPositiveDt) {
    ValidFile v;
    v.hdr.dt_s = 0.0;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "dt0.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsNonFiniteHeaderFields) {
    ValidFile v;
    v.hdr.dt_s = std::numeric_limits<double>::quiet_NaN();
    auto path = write_octt(v.hdr, v.delays, v.amps, "dt_nan.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);

    ValidFile w;
    w.hdr.fc_meas_hz = -35000.0;
    path = write_octt(w.hdr, w.delays, w.amps, "fc_neg.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsMaxDelayAboveChannelLimit) {
    ValidFile v;
    v.delays[2] = 0.250;    // > MAX_MULTIPATH_DELAY_S = 0.200
    v.hdr.max_delay_s = 0.250;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "too_long.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsNonZeroReservedHeaderBytes) {
    ValidFile v;
    v.hdr.reserved[5] = 7;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "reserved.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsNaNDelaySample) {
    ValidFile v;
    v.delays[3] = std::numeric_limits<double>::quiet_NaN();
    const auto path = write_octt(v.hdr, v.delays, v.amps, "nan_delay.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsNegativeDelaySample) {
    ValidFile v;
    v.delays[3] = -1e-9;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "neg_delay.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsNegativeAmplitudeSample) {
    ValidFile v;
    v.amps[1] = -0.1f;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "neg_amp.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsHeaderMaxDelayMismatch) {
    ValidFile v;
    v.hdr.max_delay_s = 0.050;   // data max is 0.011
    const auto path = write_octt(v.hdr, v.delays, v.amps, "max_mismatch.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsTruncatedFile) {
    ValidFile v;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "truncated.octt", 8);
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsTrailingGarbage) {
    ValidFile v;
    const auto path = write_octt(v.hdr, v.delays, v.amps, "extra.octt", 0, 16);
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
}

TEST(TapTrajectory, RejectsHeaderOnlyFile) {
    ValidFile v;
    const auto path = write_octt(v.hdr, {}, {}, "header_only.octt");
    EXPECT_THROW(TapTrajectory::load(path), TapTrajectoryError);
    // peek_header only reads the header — it must accept this file.
    EXPECT_NO_THROW(TapTrajectory::peek_header(path));
}
