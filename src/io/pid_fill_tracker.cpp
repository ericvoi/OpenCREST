#include "io/pid_fill_tracker.hpp"
#include <algorithm>

namespace openCREST {

PidFillTracker::PidFillTracker(uint32_t dac_rate, uint16_t buffer_capacity,
                               uint16_t samples_per_packet,
                               float target_fill, float kp, float ki)
    : pacer_(dac_rate, buffer_capacity, samples_per_packet,
             target_fill, kp, ki)
    , buffer_capacity_(buffer_capacity)
{}

void PidFillTracker::on_status(const protocol::StatusPayload& status,
                                time_point now) {
    pacer_.update_fill(status.buffer_fill, now);
}

void PidFillTracker::on_packet_sent(uint16_t /*packet_id*/,
                                     uint16_t /*samples*/,
                                     time_point /*now*/) {
    pacer_.notify_packet_sent();
}

bool PidFillTracker::should_send(time_point now) {
    return pacer_.should_send(now);
}

IFillTracker::time_point PidFillTracker::next_send_time() const {
    return pacer_.next_send_time();
}

uint32_t PidFillTracker::estimated_fill(time_point now) const {
    const float frac = pacer_.estimated_fill_fraction(now);
    const float fill = std::clamp(frac, 0.0f, 1.0f)
                     * static_cast<float>(buffer_capacity_);
    return static_cast<uint32_t>(fill + 0.5f);
}

void PidFillTracker::reset() {
    pacer_.reset();
}

} // namespace openCREST
