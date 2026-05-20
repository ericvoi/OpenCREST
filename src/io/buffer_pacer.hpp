#pragma once
#include <chrono>
#include <cstdint>

namespace openCREST {

// Time-based PI send-rate controller for the I/O thread's RX data stream.
//
// The previous credit/rate scheme paced via `should_send()` returning true on
// a configurable fraction of calls. That broke down because the I/O loop's
// non-send iterations are microseconds while a USB send is ~510 us, so calls
// per second were dominated by send time and the rate multiplier washed out.
// The host effectively ran open-loop at the USB ceiling with ~1% control
// authority — fine when USB happened to sit just above DAC consumption, but
// drifted monotonically when it sat just below.
//
// This controller schedules sends in absolute time:
//
//   nominal_rate = dac_rate / samples_per_packet      (target packets/sec)
//   err          = target_fill - estimated_fill_frac  (positive when low)
//   desired_rate = nominal_rate * (1 + Kp*err + Ki*∫err·dt)
//   period       = 1 / desired_rate
//   should_send: now >= next_send_time → send, advance schedule by period
//
// Integration is performed inside update_fill() using the real elapsed time
// between status reports. The integrator is bounded for anti-windup, and the
// schedule re-anchors to `now` if it falls behind to avoid bursting after a
// stall. Caller may sleep until next_send_time() to avoid busy-waiting.
class BufferPacer {
public:
    using clock      = std::chrono::steady_clock;
    using time_point = clock::time_point;
    using duration   = clock::duration;

    // dac_rate:            modem DAC sample rate (e.g. 500000)
    // buffer_capacity:     modem audio buffer size in samples (e.g. 16384)
    // samples_per_packet:  audio samples in one data packet (e.g. 255)
    // target_fill:         desired fill fraction [0,1] (default 0.5)
    // kp:                  proportional gain on normalized fill error
    // ki:                  integral gain — intentionally small so the
    //                      integrator is a slow "DAC-rate offset" correction
    //                      over seconds, not a fast millisecond controller
    BufferPacer(uint32_t dac_rate, uint16_t buffer_capacity,
                uint16_t samples_per_packet,
                float target_fill = 0.5f,
                float kp = 0.5f, float ki = 0.05f);

    // Called when a status packet provides a fresh fill measurement. Updates
    // the PI controller and resets extrapolation counters.
    void update_fill(uint16_t fill_samples);
    void update_fill(uint16_t fill_samples, time_point now);

    // Called after each successful RX data packet send (extrapolation only).
    void notify_packet_sent();

    // Returns true if an RX packet should be sent now. Advances the internal
    // schedule by one period when it returns true.
    bool should_send();
    bool should_send(time_point now);

    // Absolute time at which should_send() will next return true. Caller may
    // sleep until this point to avoid busy-waiting. Valid after the first
    // should_send() call (before that, returns the default-constructed value).
    time_point next_send_time() const { return next_send_time_; }

    // Clear all controller and schedule state. Call on transition into RX so
    // the integrator and schedule start fresh.
    void reset();

    // Current extrapolated fill as a fraction of buffer capacity (diagnostic).
    float estimated_fill_fraction() const;
    float estimated_fill_fraction(time_point now) const;

    float target_fill()  const { return target_fill_; }
    float current_rate() const { return desired_rate_; }
    float integrator()   const { return integral_; }
    float nominal_rate() const { return nominal_rate_; }

private:
    void  update_controller(uint16_t fill_samples, time_point now);
    float extrapolate(time_point now) const;

    float target_fill_;
    float kp_;
    float ki_;

    uint32_t dac_rate_;
    uint16_t buffer_capacity_;
    uint16_t samples_per_packet_;

    float nominal_rate_;     // packets/sec at perfect rate match
    float min_rate_;         // clamp bounds (±30% of nominal)
    float max_rate_;
    float max_integral_;     // anti-windup bound

    // Controller state
    float      integral_     = 0.0f;
    float      desired_rate_;
    time_point prev_update_time_{};
    bool       has_prev_update_ = false;

    // Status-report extrapolation state (diagnostic only)
    uint16_t   last_reported_fill_        = 0;
    uint32_t   packets_sent_since_report_ = 0;
    time_point last_report_time_{};
    bool       has_report_                = false;

    // Send schedule
    time_point next_send_time_{};
    bool       schedule_initialized_ = false;
};

} // namespace openCREST
