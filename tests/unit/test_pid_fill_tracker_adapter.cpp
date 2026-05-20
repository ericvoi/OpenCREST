// Tests pinning the PidFillTracker adapter's contract: it MUST be a
// transparent wrapper over BufferPacer. Any divergence in behavior would
// mean the IFillTracker abstraction silently changed PID-mode behavior,
// which is exactly what Phase 0 of the clock-tracker rollout forbids.
#include <gtest/gtest.h>
#include "io/pid_fill_tracker.hpp"
#include "io/buffer_pacer.hpp"
#include "protocol/packets.hpp"

using openCREST::PidFillTracker;
using openCREST::BufferPacer;
using openCREST::protocol::StatusPayload;
using openCREST::protocol::ControlType;
using time_point = openCREST::IFillTracker::time_point;

namespace {

constexpr uint32_t DAC_RATE        = 500'000;
constexpr uint16_t BUFFER_CAP      = 16'384;
constexpr uint16_t SAMPLES_PER_PKT = 255;

StatusPayload make_status(uint16_t fill) {
    StatusPayload s{};
    s.type                  = static_cast<uint8_t>(ControlType::STATUS);
    s.modem_state           = 1;          // RX
    s.buffer_fill           = fill;
    s.buffer_capacity       = BUFFER_CAP;
    s.attenuation_idx       = 0;
    s.error_flags           = 0;
    s.firmware_timestamp_ms = 0;
    s.rx_expected_id        = 0;
    s.fill_reference_id     = 0;
    return s;
}

} // namespace

// Test 1: PidFillTracker::should_send delegates to BufferPacer::should_send.
// Both should fire at the same wall-clock instants when given identical
// state — first call always true (schedule-init), subsequent calls track
// the configured period.
TEST(PidFillTrackerAdapter, DelegatesShouldSendToBufferPacer) {
    PidFillTracker tracker(DAC_RATE, BUFFER_CAP, SAMPLES_PER_PKT);
    BufferPacer    pacer  (DAC_RATE, BUFFER_CAP, SAMPLES_PER_PKT);

    auto t = openCREST::IFillTracker::clock::now();
    // First call after construction: schedule initializes; both true.
    EXPECT_EQ(tracker.should_send(t), pacer.should_send(t));

    // Advance by ~1 ms (well under one packet period at 500 kSPS) and
    // re-check: both should return the same answer.
    t += std::chrono::milliseconds(1);
    EXPECT_EQ(tracker.should_send(t), pacer.should_send(t));

    // Advance well past one period: both should fire again.
    t += std::chrono::milliseconds(10);
    EXPECT_EQ(tracker.should_send(t), pacer.should_send(t));
}

// Test 2: on_status(status, now) must forward buffer_fill (and nothing
// else from the status payload) into BufferPacer::update_fill(fill, now).
// We check by comparing estimated_fill after the call.
TEST(PidFillTrackerAdapter, OnStatusForwardsFillToBufferPacer) {
    PidFillTracker tracker(DAC_RATE, BUFFER_CAP, SAMPLES_PER_PKT);
    BufferPacer    pacer  (DAC_RATE, BUFFER_CAP, SAMPLES_PER_PKT);

    auto t = openCREST::IFillTracker::clock::now();
    const uint16_t fill = BUFFER_CAP / 2;

    tracker.on_status(make_status(fill), t);
    pacer.update_fill(fill, t);

    // BufferPacer returns fraction; tracker returns samples. Compare on a
    // common scale to ±1 sample (rounding).
    const uint32_t tracker_fill = tracker.estimated_fill(t);
    const uint32_t pacer_fill   =
        static_cast<uint32_t>(pacer.estimated_fill_fraction(t) * BUFFER_CAP + 0.5f);
    EXPECT_NEAR(static_cast<int32_t>(tracker_fill),
                static_cast<int32_t>(pacer_fill), 1);

    // Send-rate state should also match (delegated through the same PI).
    EXPECT_FLOAT_EQ(tracker.current_rate(), pacer.current_rate());
}

// Test 3: reset() clears BufferPacer state — integrator, schedule, and
// last-fill anchor must all return to their pre-update values.
TEST(PidFillTrackerAdapter, ResetClearsBufferPacer) {
    PidFillTracker tracker(DAC_RATE, BUFFER_CAP, SAMPLES_PER_PKT);

    auto t = openCREST::IFillTracker::clock::now();

    // Drive the controller off-nominal so reset has something to clear.
    tracker.on_status(make_status(BUFFER_CAP / 4), t);  // err = +0.25
    t += std::chrono::milliseconds(50);
    tracker.on_status(make_status(BUFFER_CAP / 4), t);  // accumulate integral
    EXPECT_GT(tracker.integrator(), 0.0f);

    tracker.reset();
    EXPECT_FLOAT_EQ(tracker.integrator(), 0.0f);
    // After reset, current_rate must match nominal again. nominal_rate is
    // dac_rate / samples_per_packet = 1960.7843...
    const float nominal = static_cast<float>(DAC_RATE) /
                          static_cast<float>(SAMPLES_PER_PKT);
    EXPECT_NEAR(tracker.current_rate(), nominal, 1e-3f);
}
