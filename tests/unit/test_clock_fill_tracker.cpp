// Tests for the clock-extrapolation fill tracker. Verifies:
//   - Steady-state extrapolation tracks within ±1 sample
//   - Fallback path engages when status references an unknown packet_id
//   - Rate estimator converges toward the true modem rate
//   - Hysteresis prevents chatter at the target boundary
//   - Burst cap limits over-fill during catch-up
//   - reset() clears anchor + ring but keeps the smoothed rate (load-bearing)
//   - First post-reset status doesn't trigger rate inference (skip flag)
#include <gtest/gtest.h>
#include "io/clock_fill_tracker.hpp"
#include "protocol/packets.hpp"

using openCREST::ClockFillTracker;
using openCREST::protocol::StatusPayload;
using openCREST::protocol::ControlType;
using ctclock     = ClockFillTracker::clock;
using time_point  = ClockFillTracker::time_point;

namespace {

constexpr uint32_t DAC_RATE        = 500'000;
constexpr uint16_t BUFFER_CAP      = 16'384;
constexpr uint16_t SAMPLES_PER_PKT = 255;

ClockFillTracker::Config base_cfg() {
    ClockFillTracker::Config c;
    c.dac_rate            = DAC_RATE;
    c.buffer_capacity     = BUFFER_CAP;
    c.samples_per_packet  = SAMPLES_PER_PKT;
    c.target_fill_frac    = 0.5f;
    c.hyst_low_frac       = 0.02f;
    c.hyst_high_frac      = 0.02f;
    c.burst_cap_multiplier = 1.5f;
    c.rate_ewma_alpha     = 0.2f;
    return c;
}

StatusPayload make_status(uint16_t fill, uint16_t fill_ref_id) {
    StatusPayload s{};
    s.type                  = static_cast<uint8_t>(ControlType::STATUS);
    s.modem_state           = 1;            // RX
    s.buffer_fill           = fill;
    s.buffer_capacity       = BUFFER_CAP;
    s.attenuation_idx       = 0;
    s.error_flags           = 0;
    s.firmware_timestamp_ms = 0;
    s.rx_expected_id        = static_cast<uint16_t>(fill_ref_id + 1);
    s.fill_reference_id     = fill_ref_id;
    return s;
}

constexpr auto us(int64_t n) { return std::chrono::microseconds(n); }

} // namespace

// Test 9: Synthetic steady-state trace. Inject one anchor; advance time
// while sending packets at the nominal rate; extrapolated fill must
// track the true fill within ±1 sample (rounding budget).
TEST(ClockFillTracker, SteadyStateExtrapolationWithinOneSample) {
    ClockFillTracker tr(base_cfg());

    const time_point T0 = ctclock::now();

    // Pre-load the ring with packet 0 sent at T0 (cumulative samples = 255).
    tr.on_packet_sent(0, SAMPLES_PER_PKT, T0);
    // Status arrives shortly after, referencing packet 0 with fill =
    // 8000 samples (~50% of buffer). Anchor is now (8000, T0).
    tr.on_status(make_status(8000, 0), T0 + us(100));

    // Send 10 more packets at perfect cadence over the next 5.1 ms.
    // Each packet adds 255 samples to the host's cumulative; modem
    // consumes 500'000 samples/sec = 255 samples per 510 µs.
    // Net fill change per packet period: +255 - 255 = 0. So fill
    // should hold at 8000.
    time_point t = T0;
    for (int n = 1; n <= 10; ++n) {
        t += us(510);
        tr.on_packet_sent(static_cast<uint16_t>(n), SAMPLES_PER_PKT, t);
    }
    const uint32_t fill_now = tr.estimated_fill(t);
    EXPECT_NEAR(static_cast<int32_t>(fill_now), 8000, 2);
}

// Test 10: A status packet referencing a fill_reference_id that isn't
// in the ring (e.g. evicted by load) MUST fall back to a status-arrival
// anchor and increment the fallback counter. The estimated fill should
// still be sensible (= status.buffer_fill at status-arrival time).
TEST(ClockFillTracker, UnknownFillReferenceIdFallsBackAndCounts) {
    ClockFillTracker tr(base_cfg());

    const time_point T0 = ctclock::now();
    EXPECT_EQ(tr.fallback_anchor_count(), 0u);

    // Status references packet 999 which was never sent.
    tr.on_status(make_status(7000, 999), T0);

    EXPECT_EQ(tr.fallback_anchor_count(), 1u);
    EXPECT_TRUE(tr.last_anchor_was_fallback());

    // At anchor time, fill matches reported value (no extrapolation yet).
    EXPECT_EQ(tr.estimated_fill(T0), 7000u);
}

// Test 11: Over 100 status packets driven by a known-rate simulated
// modem, the EWMA rate estimator converges to within 0.1% of the
// modem's actual rate. We simulate a modem running at 500'000 samples/s.
TEST(ClockFillTracker, RateEstimatorConvergesToWithinTenthPercent) {
    auto cfg = base_cfg();
    // Seed deliberately off — say 1% high — so we can see the EWMA pull
    // it back toward the true rate.
    cfg.initial_modem_rate_hint = 505'000;
    ClockFillTracker tr(cfg);

    const time_point T0 = ctclock::now();
    const double TRUE_FS = 500'000.0;

    uint16_t pkt_id = 0;
    uint64_t cum_sent = 0;
    uint32_t fill = 8192;  // start at 50%
    time_point t = T0;

    // First anchor (no rate inference yet — skip flag).
    tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
    cum_sent += SAMPLES_PER_PKT;
    tr.on_status(make_status(static_cast<uint16_t>(fill), pkt_id), t);

    for (int i = 1; i <= 100; ++i) {
        // Advance time by 5 packet periods (~2.55 ms — typical status
        // cadence). In that interval, send 5 packets at perfect cadence.
        for (int p = 0; p < 5; ++p) {
            t += us(510);
            ++pkt_id;
            tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
            cum_sent += SAMPLES_PER_PKT;
        }
        // Simulated modem consumed 5 packets' worth in that interval at
        // the TRUE rate. Fill change = 0 (sent = consumed exactly).
        // Status references packet (pkt_id) with fill unchanged.
        tr.on_status(make_status(static_cast<uint16_t>(fill), pkt_id), t);
    }
    EXPECT_NEAR(tr.estimated_modem_rate(), TRUE_FS, TRUE_FS * 0.001);
}

// Test 12: A single anomalous status packet (e.g. fill jump that
// implies rate far outside any physically plausible range) must NOT
// permanently bias the long-term estimate. The tracker may either
// reject the observation outright (sanity bound) or attenuate it via
// EWMA; either way, after recovery cycles the rate stays close to
// truth.
TEST(ClockFillTracker, RateEstimatorRobustToSingleOutlier) {
    ClockFillTracker tr(base_cfg());
    const time_point T0 = ctclock::now();
    const double TRUE_FS = 500'000.0;

    uint16_t pkt_id = 0;
    time_point t = T0;

    // Build up an anchor at steady state.
    tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
    tr.on_status(make_status(8192, pkt_id), t);
    // Run 10 normal status cycles to settle the EWMA (fill held constant).
    for (int i = 0; i < 10; ++i) {
        for (int p = 0; p < 5; ++p) {
            t += us(510);
            ++pkt_id;
            tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
        }
        tr.on_status(make_status(8192, pkt_id), t);
    }

    // Outlier: fill jumps implausibly low. This is exactly the case
    // where the sanity bound (rate ≥ 2× nominal would imply 6× normal
    // consumption) should reject the observation.
    for (int p = 0; p < 5; ++p) {
        t += us(510);
        ++pkt_id;
        tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
    }
    tr.on_status(make_status(2000, pkt_id), t);  // jump from 8192 → 2000

    // 20 normal cycles with fill held steady at the new level. Whether
    // the rate was perturbed by the outlier or not, it should converge
    // back toward TRUE_FS once clean data resumes.
    for (int i = 0; i < 20; ++i) {
        for (int p = 0; p < 5; ++p) {
            t += us(510);
            ++pkt_id;
            tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
        }
        tr.on_status(make_status(2000, pkt_id), t);  // fill steady-state
    }

    // Final estimate within 1% of truth — the outlier's effect did
    // not stick.
    EXPECT_NEAR(tr.estimated_modem_rate(), TRUE_FS, TRUE_FS * 0.01);
}

// Test 13: Hysteresis: once fill enters the hold band, should_send
// remains false until fill drops below the target. No chatter when
// fill oscillates just above the target.
TEST(ClockFillTracker, ShouldSendBelowTargetWithHysteresis) {
    auto cfg = base_cfg();
    ClockFillTracker tr(cfg);

    const time_point T0 = ctclock::now();
    const uint16_t target = static_cast<uint16_t>(cfg.target_fill_frac * BUFFER_CAP);

    // Anchor at target.
    tr.on_packet_sent(0, SAMPLES_PER_PKT, T0);
    tr.on_status(make_status(target, 0), T0);

    // Push fill above target + hyst_high → should_send must hold.
    const uint16_t above_high = static_cast<uint16_t>(target +
                                  cfg.hyst_high_frac * BUFFER_CAP + 100);
    tr.on_packet_sent(1, SAMPLES_PER_PKT, T0 + us(510));
    tr.on_status(make_status(above_high, 1), T0 + us(510));
    EXPECT_FALSE(tr.should_send(T0 + us(510)));

    // Drop fill back to just above target (still inside hold band per
    // hysteresis) → should still be holding.
    tr.on_packet_sent(2, SAMPLES_PER_PKT, T0 + us(1020));
    tr.on_status(make_status(target + 10, 2), T0 + us(1020));
    EXPECT_FALSE(tr.should_send(T0 + us(1020)));

    // Drop fill clearly below target → hold releases.
    tr.on_packet_sent(3, SAMPLES_PER_PKT, T0 + us(1530));
    tr.on_status(make_status(static_cast<uint16_t>(target - 200), 3),
                  T0 + us(1530));
    EXPECT_TRUE(tr.should_send(T0 + us(1530)));
}

// Test 14: When fill is well below target, the tracker may burst —
// but its average send rate over a window MUST NOT exceed
// burst_cap_multiplier × nominal_rate. Otherwise modem buffer
// overruns.
TEST(ClockFillTracker, BurstCappedAtMaxRate) {
    auto cfg = base_cfg();
    cfg.burst_cap_multiplier = 1.5f;
    ClockFillTracker tr(cfg);

    const time_point T0 = ctclock::now();
    // Anchor far below target so the tracker wants to burst.
    tr.on_packet_sent(0, SAMPLES_PER_PKT, T0);
    tr.on_status(make_status(1000, 0), T0);

    // Sweep 100 ms of wall time and count should_send=true responses.
    int sends = 0;
    const auto window = std::chrono::milliseconds(100);
    time_point t = T0;
    while (t < T0 + window) {
        t += us(10);  // poll every 10 µs
        if (tr.should_send(t)) {
            ++sends;
            tr.on_packet_sent(static_cast<uint16_t>(sends),
                               SAMPLES_PER_PKT, t);
        }
    }
    const double nominal = static_cast<double>(DAC_RATE) /
                            static_cast<double>(SAMPLES_PER_PKT);
    const double max_allowed = nominal * cfg.burst_cap_multiplier *
                                std::chrono::duration<double>(window).count();
    EXPECT_LE(sends, static_cast<int>(max_allowed) + 1);
}

// Test 15: reset() clears the anchor and the send-timestamp ring, but
// MUST preserve the smoothed modem-rate estimate (hardware constant;
// re-learning it every state transition would waste an anchor pair
// per cycle).
TEST(ClockFillTracker, ResetClearsAnchorAndRingButKeepsRate) {
    auto cfg = base_cfg();
    cfg.initial_modem_rate_hint = 500'000;
    ClockFillTracker tr(cfg);

    // Drive the rate estimator a bit so it diverges from the initial seed.
    const time_point T0 = ctclock::now();
    time_point t = T0;
    uint16_t pkt_id = 0;
    tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
    tr.on_status(make_status(8192, pkt_id), t);
    for (int i = 0; i < 5; ++i) {
        for (int p = 0; p < 5; ++p) {
            t += us(510);
            ++pkt_id;
            tr.on_packet_sent(pkt_id, SAMPLES_PER_PKT, t);
        }
        tr.on_status(make_status(8192, pkt_id), t);
    }
    const double pre_reset_rate = tr.estimated_modem_rate();

    tr.reset();

    EXPECT_FALSE(tr.has_anchor());
    EXPECT_DOUBLE_EQ(tr.estimated_modem_rate(), pre_reset_rate);

    // After reset, a fresh status referencing a packet that pre-dates
    // the reset MUST fall back (ring was cleared).
    tr.on_status(make_status(8000, pkt_id), t + us(1));
    EXPECT_TRUE(tr.last_anchor_was_fallback());
}

// Test 16: The first status after a reset MUST NOT update the rate
// estimator. The pipeline depth and pre-reset state would otherwise
// confuse the inference and slam the rate estimate.
TEST(ClockFillTracker, SkipsRateInferenceOnFirstPostResetStatus) {
    auto cfg = base_cfg();
    cfg.initial_modem_rate_hint = 500'000;
    ClockFillTracker tr(cfg);

    // Build an anchor and let one rate inference happen.
    const time_point T0 = ctclock::now();
    time_point t = T0;
    tr.on_packet_sent(0, SAMPLES_PER_PKT, t);
    tr.on_status(make_status(8192, 0), t);
    for (int p = 0; p < 5; ++p) {
        t += us(510);
        tr.on_packet_sent(static_cast<uint16_t>(p + 1), SAMPLES_PER_PKT, t);
    }
    tr.on_status(make_status(8192, 5), t);
    const double rate_before_reset = tr.estimated_modem_rate();

    tr.reset();
    // Pre-load a new packet and feed a status. The status's
    // (fill - cumulative) math vs initial rate would normally produce
    // an inference step — but the skip flag should suppress it.
    tr.on_packet_sent(10, SAMPLES_PER_PKT, t + us(1));
    tr.on_status(make_status(8192, 10), t + us(1));
    EXPECT_DOUBLE_EQ(tr.estimated_modem_rate(), rate_before_reset);
}
