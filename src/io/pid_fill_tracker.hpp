#pragma once
#include "core/fill_tracker.hpp"
#include "io/buffer_pacer.hpp"

namespace openCREST {

// IFillTracker implementation that delegates to the legacy BufferPacer
// PI controller. Behavior is bit-for-bit identical to using BufferPacer
// directly — this class exists only to fit BufferPacer behind the
// IFillTracker interface so ModemIO can select between PID and
// clock-extrapolation trackers at compile time.
class PidFillTracker final : public IFillTracker {
public:
    PidFillTracker(uint32_t dac_rate, uint16_t buffer_capacity,
                   uint16_t samples_per_packet,
                   float target_fill = 0.5f,
                   float kp = 0.5f, float ki = 0.05f);

    void on_status(const protocol::StatusPayload& status,
                   time_point now) override;

    void on_packet_sent(uint16_t packet_id, uint16_t samples,
                        time_point now) override;

    bool should_send(time_point now) override;

    time_point next_send_time() const override;

    uint32_t estimated_fill(time_point now) const override;

    void reset() override;

    // Diagnostic accessors mirroring BufferPacer's, for tests and metrics.
    float target_fill()  const { return pacer_.target_fill(); }
    float current_rate() const { return pacer_.current_rate(); }
    float integrator()   const { return pacer_.integrator(); }

private:
    BufferPacer pacer_;
    uint16_t    buffer_capacity_;
};

} // namespace openCREST
