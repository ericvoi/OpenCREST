#pragma once
#include <chrono>
#include <cstdint>

namespace openCREST {

// Time-based PI send-rate controller for the I/O thread's RX data stream.
//
//   nominal_rate = dac_rate / samples_per_packet      (target packets/sec)
//   err          = target_fill - estimated_fill_frac  (positive when low)
//   desired_rate = nominal_rate * (1 + Kp*err + Ki*∫err·dt)
//   period       = 1 / desired_rate
//   should_send: now >= next_send_time → send, advance schedule by period
//
// Integrator is bounded; schedule re-anchors to `now` after a stall to
// avoid bursting. Caller may sleep until next_send_time() to avoid
// busy-waiting.
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
    // ki:                  integral gain — kept small so the integrator
    //                      acts as a slow DAC-rate offset correction.
    BufferPacer(uint32_t dac_rate, uint16_t buffer_capacity,
                uint16_t samples_per_packet,
                float target_fill = 0.5f,
                float kp = 0.5f, float ki = 0.05f);

    // Apply a fresh fill measurement from a status packet.
    void update_fill(uint16_t fill_samples);
    void update_fill(uint16_t fill_samples, time_point now);

    // Notify that one RX data packet was sent (drives extrapolation only).
    void notify_packet_sent();

    // True if an RX packet should be sent now. Advances the schedule by
    // one period on a true return.
    bool should_send();
    bool should_send(time_point now);

    // Absolute time of the next scheduled send. Caller may sleep until
    // this point. Default-constructed before the first should_send() call.
    time_point next_send_time() const { return next_send_time_; }

    // Reset controller and schedule state. Call on transition into RX.
    void reset();

    // Extrapolated fill as a fraction of buffer capacity (diagnostic).
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
