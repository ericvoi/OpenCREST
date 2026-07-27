#pragma once

// Minimal .octt writer for tests: builds valid trajectory files without
// going through Python. Layout mirrors experiments/lib/tap_trajectory.py.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace test_helpers {

// delays/amps are frame-major: frame_count * tap_count entries.
inline void write_octt_file(const std::string& path,
                            uint32_t tap_count, uint32_t frame_count,
                            double dt_s, double fc_meas_hz,
                            const std::vector<double>& delays,
                            const std::vector<float>&  amps) {
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

    RawHeader hdr;
    hdr.tap_count   = tap_count;
    hdr.frame_count = frame_count;
    hdr.dt_s        = dt_s;
    hdr.fc_meas_hz  = fc_meas_hz;
    hdr.max_delay_s = *std::max_element(delays.begin(), delays.end());

    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    for (size_t i = 0; i < delays.size(); ++i) {
        RawRecord rec;
        rec.delay_s   = delays[i];
        rec.amplitude = amps[i];
        f.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }
}

} // namespace test_helpers
