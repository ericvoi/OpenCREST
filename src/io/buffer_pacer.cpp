#include "io/buffer_pacer.hpp"
#include <algorithm>

namespace openCREST {

BufferPacer::BufferPacer(uint32_t dac_rate, uint16_t buffer_capacity,
                         uint16_t samples_per_packet,
                         float target_fill, float kp, float ki)
    : target_fill_(target_fill)
    , kp_(kp)
    , ki_(ki)
    , dac_rate_(dac_rate)
    , buffer_capacity_(buffer_capacity)
    , samples_per_packet_(samples_per_packet)
    , nominal_rate_(static_cast<float>(dac_rate) /
                    static_cast<float>(samples_per_packet))
    , min_rate_(nominal_rate_ * 0.7f)
    , max_rate_(nominal_rate_ * 1.3f)
    , max_integral_(0.2f)
    , desired_rate_(nominal_rate_)
{}

// ---------------------------------------------------------------------------
// Status update + PI step
// ---------------------------------------------------------------------------

void BufferPacer::update_fill(uint16_t fill_samples) {
    update_fill(fill_samples, clock::now());
}

void BufferPacer::update_fill(uint16_t fill_samples, time_point now) {
    last_reported_fill_        = fill_samples;
    packets_sent_since_report_ = 0;
    last_report_time_          = now;
    has_report_                = true;

    update_controller(fill_samples, now);
}

void BufferPacer::update_controller(uint16_t fill_samples, time_point now) {
    const float fill_frac = static_cast<float>(fill_samples) /
                            static_cast<float>(buffer_capacity_);
    const float err = target_fill_ - fill_frac;  // positive when below target

    if (has_prev_update_) {
        // Real elapsed time between status reports. Cap to bound the step
        // when the gap is unusually long (e.g. just after a state transition).
        const float dt = std::chrono::duration<float>(
            now - prev_update_time_).count();
        const float dt_clamped = std::min(dt, 0.1f);
        integral_ += err * dt_clamped;
        integral_ = std::clamp(integral_, -max_integral_, max_integral_);
    }
    has_prev_update_  = true;
    prev_update_time_ = now;

    desired_rate_ = nominal_rate_ * (1.0f + kp_ * err + ki_ * integral_);
    desired_rate_ = std::clamp(desired_rate_, min_rate_, max_rate_);
}

// ---------------------------------------------------------------------------
// Reset (called on transition into RX)
// ---------------------------------------------------------------------------

void BufferPacer::reset() {
    has_report_                = false;
    packets_sent_since_report_ = 0;
    integral_                  = 0.0f;
    desired_rate_              = nominal_rate_;
    has_prev_update_           = false;
    schedule_initialized_      = false;
}

// ---------------------------------------------------------------------------
// Packet-sent notification
// ---------------------------------------------------------------------------

void BufferPacer::notify_packet_sent() {
    ++packets_sent_since_report_;
}

// ---------------------------------------------------------------------------
// Extrapolation (diagnostic only — controller updates from status reports)
// ---------------------------------------------------------------------------

float BufferPacer::extrapolate(time_point now) const {
    if (!has_report_) return target_fill_;

    const auto elapsed =
        std::chrono::duration<float>(now - last_report_time_);
    const float dac_consumed = elapsed.count() * static_cast<float>(dac_rate_);
    const float host_sent    = static_cast<float>(packets_sent_since_report_)
                             * static_cast<float>(samples_per_packet_);

    float estimated = static_cast<float>(last_reported_fill_)
                    - dac_consumed
                    + host_sent;

    estimated = std::clamp(estimated, 0.0f,
                           static_cast<float>(buffer_capacity_));

    return estimated / static_cast<float>(buffer_capacity_);
}

float BufferPacer::estimated_fill_fraction() const {
    return extrapolate(clock::now());
}

float BufferPacer::estimated_fill_fraction(time_point now) const {
    return extrapolate(now);
}

// ---------------------------------------------------------------------------
// Time-based send decision
// ---------------------------------------------------------------------------

bool BufferPacer::should_send() {
    return should_send(clock::now());
}

bool BufferPacer::should_send(time_point now) {
    if (!schedule_initialized_) {
        next_send_time_       = now;
        schedule_initialized_ = true;
    }
    if (now < next_send_time_) return false;

    const auto period = std::chrono::duration<double>(1.0 / desired_rate_);
    const auto period_d = std::chrono::duration_cast<duration>(period);
    next_send_time_ += period_d;

    // Bounded catch-up. If next_send_time_ is still in the past after the
    // advance, the caller fell behind (typically: libusb poll blocked the
    // I/O loop for ~1 ms). Allow up to MAX_CATCHUP_SLOTS of the backlog to
    // be redeemed as back-to-back sends — the old re-anchor forfeited those
    // slots permanently, which capped throughput below nominal whenever poll
    // stalls ate real time. Beyond that, re-anchor to avoid bursting through
    // a large backlog that would over-fill the modem ring.
    constexpr int MAX_CATCHUP_SLOTS = 3;
    if (now - next_send_time_ > period_d * MAX_CATCHUP_SLOTS) {
        next_send_time_ = now + period_d;
    }
    return true;
}

} // namespace openCREST
