#include "channel/model/replay/tap_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

#include "core/constants.hpp"

namespace openCREST {

namespace {

// On-disk layout, little-endian (writer: experiments/lib/tap_trajectory.py).
struct RawHeader {
    char     magic[4];
    uint32_t version;
    uint32_t tap_count;
    uint32_t frame_count;
    double   dt_s;
    double   fc_meas_hz;
    double   max_delay_s;
    uint8_t  reserved[24];
};
static_assert(sizeof(RawHeader) == 64);

struct RawRecord {
    double delay_s;
    float  amplitude;
    float  reserved;
};
static_assert(sizeof(RawRecord) == 16);

[[noreturn]] void fail(const std::string& path, const std::string& what) {
    throw TapTrajectoryError(path + ": " + what);
}

RawHeader read_and_validate_header(std::ifstream& f, const std::string& path) {
    RawHeader raw{};
    f.read(reinterpret_cast<char*>(&raw), sizeof(raw));
    if (f.gcount() != static_cast<std::streamsize>(sizeof(raw))) {
        fail(path, "truncated header");
    }
    if (std::memcmp(raw.magic, "OCTT", 4) != 0) {
        fail(path, "bad magic (not an .octt file)");
    }
    if (raw.version != 1) {
        fail(path, "unsupported version " + std::to_string(raw.version));
    }
    if (raw.tap_count < 1 || raw.tap_count > MAX_TAPS_PER_CHANNEL) {
        fail(path, "tap_count " + std::to_string(raw.tap_count)
                   + " out of range 1.." + std::to_string(MAX_TAPS_PER_CHANNEL));
    }
    if (raw.frame_count < 2) {
        fail(path, "frame_count " + std::to_string(raw.frame_count) + " < 2");
    }
    if (!std::isfinite(raw.dt_s) || raw.dt_s <= 0.0) {
        fail(path, "dt_s must be finite and > 0");
    }
    if (!std::isfinite(raw.fc_meas_hz) || raw.fc_meas_hz <= 0.0) {
        fail(path, "fc_meas_hz must be finite and > 0");
    }
    if (!std::isfinite(raw.max_delay_s) || raw.max_delay_s < 0.0) {
        fail(path, "max_delay_s must be finite and >= 0");
    }
    if (raw.max_delay_s > static_cast<double>(MAX_MULTIPATH_DELAY_S)) {
        fail(path, "max_delay_s " + std::to_string(raw.max_delay_s)
                   + " exceeds the "
                   + std::to_string(MAX_MULTIPATH_DELAY_S) + " s channel limit");
    }
    for (const uint8_t b : raw.reserved) {
        if (b != 0) fail(path, "nonzero reserved header bytes");
    }
    return raw;
}

std::ifstream open_binary(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) fail(path, "cannot open file");
    return f;
}

} // namespace

TapTrajectory::Header TapTrajectory::peek_header(const std::string& path) {
    auto f = open_binary(path);
    const RawHeader raw = read_and_validate_header(f, path);
    return Header{raw.tap_count, raw.frame_count,
                  raw.dt_s, raw.fc_meas_hz, raw.max_delay_s};
}

TapTrajectory TapTrajectory::load(const std::string& path) {
    auto f = open_binary(path);
    const RawHeader raw = read_and_validate_header(f, path);

    const size_t n_records =
        static_cast<size_t>(raw.frame_count) * raw.tap_count;

    f.seekg(0, std::ios::end);
    const auto file_size = static_cast<size_t>(f.tellg());
    const size_t expected = sizeof(RawHeader) + n_records * sizeof(RawRecord);
    if (file_size != expected) {
        fail(path, "file size " + std::to_string(file_size)
                   + " != expected " + std::to_string(expected));
    }
    f.seekg(sizeof(RawHeader), std::ios::beg);

    std::vector<RawRecord> records(n_records);
    f.read(reinterpret_cast<char*>(records.data()),
           static_cast<std::streamsize>(n_records * sizeof(RawRecord)));
    if (f.gcount()
        != static_cast<std::streamsize>(n_records * sizeof(RawRecord))) {
        fail(path, "truncated record data");
    }

    TapTrajectory traj;
    traj.header_ = Header{raw.tap_count, raw.frame_count,
                          raw.dt_s, raw.fc_meas_hz, raw.max_delay_s};
    traj.delays_.resize(n_records);
    traj.amplitudes_.resize(n_records);

    double data_max_delay = 0.0;
    for (size_t i = 0; i < n_records; ++i) {
        const auto& rec = records[i];
        if (!std::isfinite(rec.delay_s) || rec.delay_s < 0.0) {
            fail(path, "invalid delay sample at record " + std::to_string(i));
        }
        if (!std::isfinite(rec.amplitude) || rec.amplitude < 0.0f) {
            fail(path, "invalid amplitude sample at record "
                       + std::to_string(i));
        }
        data_max_delay      = std::max(data_max_delay, rec.delay_s);
        traj.delays_[i]     = rec.delay_s;
        traj.amplitudes_[i] = rec.amplitude;
    }
    if (std::abs(data_max_delay - raw.max_delay_s) > 1e-6) {
        fail(path, "header max_delay_s " + std::to_string(raw.max_delay_s)
                   + " != data max " + std::to_string(data_max_delay));
    }

    return traj;
}

TapTrajectory::Sample TapTrajectory::sample(size_t tap,
                                            double t_record_s) const {
    const size_t taps = header_.tap_count;
    const size_t last = header_.frame_count - 1;

    const double u_raw = t_record_s / header_.dt_s;
    const double u = std::clamp(u_raw, 0.0, static_cast<double>(last));
    const size_t i = std::min(static_cast<size_t>(u), last - 1);
    const double mu = u - static_cast<double>(i);

    // Clamped endpoints: first/last frames virtually duplicated.
    const size_t f0 = (i == 0) ? 0 : i - 1;
    const size_t f1 = i;
    const size_t f2 = i + 1;
    const size_t f3 = std::min(i + 2, last);

    const auto interp = [&](const double p0, const double p1,
                            const double p2, const double p3) {
        // Same Horner form as SourceDelayLine::read_at.
        return p1 + 0.5 * mu * ((p2 - p0)
            + mu * ((2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3)
                + mu * (3.0 * (p1 - p2) + p3 - p0)));
    };

    const double delay = interp(delays_[f0 * taps + tap],
                                delays_[f1 * taps + tap],
                                delays_[f2 * taps + tap],
                                delays_[f3 * taps + tap]);
    const double amp   = interp(amplitudes_[f0 * taps + tap],
                                amplitudes_[f1 * taps + tap],
                                amplitudes_[f2 * taps + tap],
                                amplitudes_[f3 * taps + tap]);

    return Sample{delay, static_cast<float>(std::max(amp, 0.0))};
}

} // namespace openCREST
