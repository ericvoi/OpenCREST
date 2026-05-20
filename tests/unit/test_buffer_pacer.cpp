#include <gtest/gtest.h>
#include "io/buffer_pacer.hpp"

using openCREST::BufferPacer;
using steady     = BufferPacer::clock;
using time_point = BufferPacer::time_point;

static constexpr uint32_t DAC_RATE          = 500'000;
static constexpr uint16_t BUFFER_CAP        = 16'384;
static constexpr uint16_t SAMPLES_PER_PKT   = 255;
static constexpr float    TARGET_FILL       = 0.5f;
static constexpr float    KP                = 0.5f;
static constexpr float    KI                = 0.05f;

static constexpr float NOMINAL_RATE =
    static_cast<float>(DAC_RATE) / static_cast<float>(SAMPLES_PER_PKT);

static BufferPacer make_pacer(float target = TARGET_FILL,
                              float kp = KP, float ki = KI) {
    return BufferPacer(DAC_RATE, BUFFER_CAP, SAMPLES_PER_PKT, target, kp, ki);
}

template <class Rep, class Period>
static time_point advance(time_point t,
                          std::chrono::duration<Rep, Period> d) {
    return t + std::chrono::duration_cast<time_point::duration>(d);
}

// ---------------------------------------------------------------------------
// Initial state
// ---------------------------------------------------------------------------

TEST(BufferPacer, BeforeFirstUpdateReturnsTargetFill) {
    auto pacer = make_pacer();
    auto t = steady::now();
    EXPECT_FLOAT_EQ(pacer.estimated_fill_fraction(t), TARGET_FILL);
}

TEST(BufferPacer, FirstShouldSendReturnsTrue) {
    auto pacer = make_pacer();
    auto t = steady::now();
    EXPECT_TRUE(pacer.should_send(t));
}

TEST(BufferPacer, InitialRateMatchesNominal) {
    auto pacer = make_pacer();
    EXPECT_NEAR(pacer.current_rate(), NOMINAL_RATE, 1e-3f);
}

// ---------------------------------------------------------------------------
// Extrapolation
// ---------------------------------------------------------------------------

TEST(BufferPacer, FillMatchesReportedImmediatelyAfterUpdate) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t);
    EXPECT_NEAR(pacer.estimated_fill_fraction(t), 0.5f, 1e-5f);
}

TEST(BufferPacer, FillDecreasesDueToDacConsumption) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t0);

    auto t1 = advance(t0, std::chrono::microseconds(1000));
    float frac = pacer.estimated_fill_fraction(t1);

    float expected = static_cast<float>(BUFFER_CAP / 2 - 500) / BUFFER_CAP;
    EXPECT_NEAR(frac, expected, 1e-3f);
}

TEST(BufferPacer, FillIncreasesDueToSentPackets) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t);

    pacer.notify_packet_sent();
    pacer.notify_packet_sent();

    float frac = pacer.estimated_fill_fraction(t);
    float expected =
        static_cast<float>(BUFFER_CAP / 2 + 2 * SAMPLES_PER_PKT) / BUFFER_CAP;
    EXPECT_NEAR(frac, expected, 1e-5f);
}

TEST(BufferPacer, ExtrapolationCombinesConsumptionAndSends) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    pacer.update_fill(8192, t0);

    pacer.notify_packet_sent();
    pacer.notify_packet_sent();
    pacer.notify_packet_sent();

    auto t1 = advance(t0, std::chrono::microseconds(2000));
    float frac = pacer.estimated_fill_fraction(t1);

    float expected = static_cast<float>(8192 - 1000 + 765) / BUFFER_CAP;
    EXPECT_NEAR(frac, expected, 1e-3f);
}

TEST(BufferPacer, ExtrapolationClampsToZero) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    pacer.update_fill(100, t0);
    auto t1 = advance(t0, std::chrono::microseconds(10'000));
    EXPECT_FLOAT_EQ(pacer.estimated_fill_fraction(t1), 0.0f);
}

TEST(BufferPacer, ExtrapolationClampsToOne) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    pacer.update_fill(BUFFER_CAP - 10, t0);
    for (int i = 0; i < 100; ++i) pacer.notify_packet_sent();
    EXPECT_FLOAT_EQ(pacer.estimated_fill_fraction(t0), 1.0f);
}

TEST(BufferPacer, UpdateFillResetsExtrapolationState) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    pacer.update_fill(8192, t0);
    for (int i = 0; i < 10; ++i) pacer.notify_packet_sent();

    auto t1 = advance(t0, std::chrono::microseconds(1000));
    pacer.update_fill(8000, t1);
    EXPECT_NEAR(pacer.estimated_fill_fraction(t1),
                8000.0f / BUFFER_CAP, 1e-5f);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

TEST(BufferPacer, ResetReturnsToTargetFill) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    pacer.update_fill(1000, t0);
    pacer.notify_packet_sent();

    pacer.reset();

    auto t1 = advance(t0, std::chrono::microseconds(500));
    EXPECT_FLOAT_EQ(pacer.estimated_fill_fraction(t1), TARGET_FILL);
}

TEST(BufferPacer, ResetClearsIntegratorAndRate) {
    auto pacer = make_pacer();
    auto t0 = steady::now();

    // Strong below-target error sustained over several updates → integrator grows.
    for (int i = 0; i < 10; ++i) {
        auto t = advance(t0, std::chrono::milliseconds(5 * (i + 1)));
        pacer.update_fill(1000, t);
    }
    EXPECT_GT(pacer.integrator(), 0.0f);
    EXPECT_GT(pacer.current_rate(), NOMINAL_RATE);

    pacer.reset();
    EXPECT_FLOAT_EQ(pacer.integrator(),   0.0f);
    EXPECT_NEAR   (pacer.current_rate(),  NOMINAL_RATE, 1e-3f);
}

// ---------------------------------------------------------------------------
// PI controller behavior
// ---------------------------------------------------------------------------

TEST(BufferPacer, LowFillIncreasesRateAboveNominal) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 4, t);  // 25%, error = +0.25
    EXPECT_GT(pacer.current_rate(), NOMINAL_RATE);
}

TEST(BufferPacer, HighFillDecreasesRateBelowNominal) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP * 3 / 4, t);  // 75%, error = -0.25
    EXPECT_LT(pacer.current_rate(), NOMINAL_RATE);
}

TEST(BufferPacer, AtTargetFillRateMatchesNominal) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t);
    EXPECT_NEAR(pacer.current_rate(), NOMINAL_RATE, 1e-3f);
}

TEST(BufferPacer, RateClampedAtUpperBoundWhenBufferEmpty) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    // Drive integrator hard while reporting empty.
    for (int i = 0; i < 100; ++i) {
        auto t = advance(t0, std::chrono::milliseconds(5 * (i + 1)));
        pacer.update_fill(0, t);
    }
    EXPECT_LE(pacer.current_rate(), NOMINAL_RATE * 1.31f);
    EXPECT_GT(pacer.current_rate(), NOMINAL_RATE);
}

TEST(BufferPacer, RateClampedAtLowerBoundWhenBufferFull) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    for (int i = 0; i < 100; ++i) {
        auto t = advance(t0, std::chrono::milliseconds(5 * (i + 1)));
        pacer.update_fill(BUFFER_CAP, t);
    }
    EXPECT_GE(pacer.current_rate(), NOMINAL_RATE * 0.69f);
    EXPECT_LT(pacer.current_rate(), NOMINAL_RATE);
}

TEST(BufferPacer, IntegratorAccumulatesUnderConstantError) {
    auto pacer = make_pacer();
    auto t0 = steady::now();

    // First update: integrator still 0 (no prior dt) — only proportional term.
    pacer.update_fill(BUFFER_CAP / 4, t0);
    const float rate0 = pacer.current_rate();

    for (int i = 1; i <= 10; ++i) {
        auto t = advance(t0, std::chrono::milliseconds(5 * i));
        pacer.update_fill(BUFFER_CAP / 4, t);
    }
    EXPECT_GT(pacer.current_rate(), rate0);
    EXPECT_GT(pacer.integrator(),   0.0f);
}

TEST(BufferPacer, IntegratorBoundedForAntiWindup) {
    auto pacer = make_pacer();
    auto t0 = steady::now();
    for (int i = 0; i < 1000; ++i) {
        auto t = advance(t0, std::chrono::milliseconds(10 * (i + 1)));
        pacer.update_fill(0, t);
    }
    EXPECT_LE(pacer.integrator(), 0.21f);
}

// ---------------------------------------------------------------------------
// Time-based scheduling
// ---------------------------------------------------------------------------

TEST(BufferPacer, ShouldSendPacesAtNominalPeriodAtTargetFill) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t);

    EXPECT_TRUE(pacer.should_send(t));

    const auto period = std::chrono::microseconds(
        static_cast<long>(1e6f / NOMINAL_RATE));

    EXPECT_FALSE(pacer.should_send(t + period / 2));
    EXPECT_TRUE (pacer.should_send(t + period + std::chrono::microseconds(5)));
}

TEST(BufferPacer, ShouldSendPacesFasterWhenFillLow) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(0, t);  // empty → rate clamped to upper bound

    EXPECT_TRUE(pacer.should_send(t));

    // Period at upper-bound rate is < nominal period.
    const auto nominal_period = std::chrono::microseconds(
        static_cast<long>(1e6f / NOMINAL_RATE));

    EXPECT_TRUE(pacer.should_send(t + nominal_period));
}

TEST(BufferPacer, ScheduleReanchorsWhenFarBehind) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t);

    pacer.should_send(t);

    // Big jump forward (100 ms ≫ MAX_CATCHUP_SLOTS periods). The pacer must
    // re-anchor rather than burst through the backlog — one send allowed,
    // the next should_send at the same wall-clock instant returns false.
    auto t1 = advance(t, std::chrono::milliseconds(100));
    EXPECT_TRUE (pacer.should_send(t1));
    EXPECT_FALSE(pacer.should_send(t1));
}

TEST(BufferPacer, CatchUpAllowsSmallBurstAfterStall) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t);

    // Consume the first scheduled send at t. next_send_time is now t+period.
    pacer.should_send(t);

    // Simulate a 3-period stall (e.g. a libusb poll that blocked the I/O
    // loop for ~3 send slots). On resume, the pacer should allow us to send
    // back-to-back to absorb the backlog rather than forfeiting slots.
    const auto period = std::chrono::microseconds(
        static_cast<long>(1e6f / NOMINAL_RATE));
    auto t1 = t + period * 3;

    int catchup_sends = 0;
    while (pacer.should_send(t1) && catchup_sends < 10) ++catchup_sends;

    EXPECT_GE(catchup_sends, 2);  // at least two back-to-back sends
    EXPECT_LE(catchup_sends, 4);  // bounded — no runaway burst
}

TEST(BufferPacer, NextSendTimeAdvancesOnEachSend) {
    auto pacer = make_pacer();
    auto t = steady::now();
    pacer.update_fill(BUFFER_CAP / 2, t);

    pacer.should_send(t);
    auto t1 = pacer.next_send_time();
    pacer.should_send(t1);
    auto t2 = pacer.next_send_time();
    EXPECT_GT(t2, t1);
}
