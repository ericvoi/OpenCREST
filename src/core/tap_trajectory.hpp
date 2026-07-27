#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace openCREST {

class TapTrajectoryError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Per-tap delay/amplitude trajectories on a uniform time grid, loaded from
// an .octt file. The file is produced offline (experiments/lib/
// tap_trajectory.py); this class owns the read-side contract:
//
//  - Tap state between frames is Catmull-Rom interpolated on the uniform
//    grid with clamped endpoints (first/last frames virtually duplicated).
//  - Query time is clamped to [0, duration_s()].
//  - Interpolated amplitude is clamped to >= 0 (Catmull-Rom can undershoot
//    around a fade; negative amplitude is meaningless — phase is encoded
//    in the delay track).
//  - Delay is returned as interpolated. The converter guarantees the
//    interpolated track stays above a positive guard floor; raw samples
//    are validated >= 0 at load.
//
// load() is init-time only (file I/O, allocation); sample() is const,
// zero-allocation, and safe on the processing hot path.
class TapTrajectory {
public:
    struct Header {
        uint32_t tap_count   = 0;
        uint32_t frame_count = 0;
        double   dt_s        = 0.0;
        double   fc_meas_hz  = 0.0;
        double   max_delay_s = 0.0;
    };

    struct Sample {
        double delay_s   = 0.0;   // excess over bulk propagation delay
        float  amplitude = 0.0f;  // linear, >= 0
    };

    // Validate and read only the 64-byte header (used by the scenario
    // loader for sizing without pulling the whole file).
    static Header peek_header(const std::string& path);

    // Load and fully validate. Throws TapTrajectoryError with a
    // path-prefixed message on any malformed input.
    static TapTrajectory load(const std::string& path);

    size_t tap_count()   const { return header_.tap_count; }
    size_t frame_count() const { return header_.frame_count; }
    double dt_s()        const { return header_.dt_s; }
    double fc_meas_hz()  const { return header_.fc_meas_hz; }
    double max_delay_s() const { return header_.max_delay_s; }
    double duration_s()  const {
        return static_cast<double>(header_.frame_count - 1) * header_.dt_s;
    }

    // Tap state at record time t_record_s. tap must be < tap_count().
    Sample sample(size_t tap, double t_record_s) const;

private:
    Header header_{};
    std::vector<double> delays_;      // frame-major: [frame * tap_count + tap]
    std::vector<float>  amplitudes_;  // frame-major
};

} // namespace openCREST
