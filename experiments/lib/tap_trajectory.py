"""Reader/writer for openCREST tap-trajectory files (``.octt``).

An ``.octt`` file carries per-tap delay/amplitude trajectories sampled on a
uniform time grid. It is the interchange format between the offline TVIR
converter (Python) and the replay channel mode (C++). Layout, little-endian:

Header (64 bytes)::

    offset  type      field
    0       char[4]   magic "OCTT"
    4       u32       version (1)
    8       u32       tap_count      (1..32)
    12      u32       frame_count    (>= 2)
    16      f64       dt_s           (> 0; frame i is record time i*dt_s)
    24      f64       fc_meas_hz     (> 0; measurement center frequency)
    32      f64       max_delay_s    (max over all delay samples)
    40      u8[24]    reserved, zero

Data: ``frame_count x tap_count`` records, frame-major::

    f64  delay_s      (excess over bulk propagation delay, >= 0, finite)
    f32  amplitude    (linear, >= 0; phase is encoded in the delay track)
    f32  reserved, zero

Tap state between frames is Catmull-Rom interpolated on the uniform grid
with clamped endpoints (first/last frames virtually duplicated).
:func:`catmull_rom_uniform` is the reference implementation the C++ side
must match.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAGIC = b"OCTT"
VERSION = 1
HEADER_SIZE = 64
HEADER_FMT = "<4sIIIddd24s"
RECORD_DTYPE = np.dtype([("delay_s", "<f8"),
                         ("amplitude", "<f4"),
                         ("reserved", "<f4")])

MAX_TAPS = 32              # core/constants.hpp MAX_TAPS_PER_CHANNEL
MAX_DELAY_S = 0.200        # core/constants.hpp MAX_MULTIPATH_DELAY_S


@dataclass
class TapTrajectoryData:
    dt_s: float
    fc_meas_hz: float
    delays: np.ndarray      # (frame_count, tap_count) float64, seconds
    amplitudes: np.ndarray  # (frame_count, tap_count) float32, linear

    @property
    def tap_count(self) -> int:
        return self.delays.shape[1]

    @property
    def frame_count(self) -> int:
        return self.delays.shape[0]

    @property
    def duration_s(self) -> float:
        return (self.frame_count - 1) * self.dt_s


def write_octt(path: str | Path, dt_s: float, fc_meas_hz: float,
               delays: np.ndarray, amplitudes: np.ndarray) -> None:
    """Validate and write an ``.octt`` file.

    ``delays``/``amplitudes`` are ``(frame_count, tap_count)`` arrays;
    delays in seconds (excess over the bulk propagation delay).
    """
    delays = np.asarray(delays, dtype=np.float64)
    amplitudes = np.asarray(amplitudes, dtype=np.float32)

    if delays.ndim != 2:
        raise ValueError(f"delays must be 2-D (frames, taps), got {delays.shape}")
    if amplitudes.shape != delays.shape:
        raise ValueError(f"shape mismatch: delays {delays.shape} vs "
                         f"amplitudes {amplitudes.shape}")
    frame_count, tap_count = delays.shape
    if not 1 <= tap_count <= MAX_TAPS:
        raise ValueError(f"tap_count must be 1..{MAX_TAPS}, got {tap_count}")
    if frame_count < 2:
        raise ValueError(f"frame_count must be >= 2, got {frame_count}")
    if not (dt_s > 0.0 and np.isfinite(dt_s)):
        raise ValueError(f"dt_s must be finite and > 0, got {dt_s}")
    if not (fc_meas_hz > 0.0 and np.isfinite(fc_meas_hz)):
        raise ValueError(f"fc_meas_hz must be finite and > 0, got {fc_meas_hz}")
    if not np.all(np.isfinite(delays)):
        raise ValueError("delays contain non-finite values")
    if not np.all(np.isfinite(amplitudes)):
        raise ValueError("amplitudes contain non-finite values")
    if np.any(delays < 0.0):
        raise ValueError("delays must be >= 0")
    if np.any(amplitudes < 0.0):
        raise ValueError("amplitudes must be >= 0 (phase lives in the delay)")
    max_delay_s = float(delays.max())
    if max_delay_s > MAX_DELAY_S:
        raise ValueError(f"max delay {max_delay_s:.6f} s exceeds the "
                         f"{MAX_DELAY_S} s channel limit")

    header = struct.pack(HEADER_FMT, MAGIC, VERSION, tap_count, frame_count,
                         float(dt_s), float(fc_meas_hz), max_delay_s,
                         b"\x00" * 24)
    records = np.zeros(frame_count * tap_count, dtype=RECORD_DTYPE)
    records["delay_s"] = delays.reshape(-1)
    records["amplitude"] = amplitudes.reshape(-1)

    with Path(path).open("wb") as f:
        f.write(header)
        records.tofile(f)


def read_octt(path: str | Path) -> TapTrajectoryData:
    """Read and validate an ``.octt`` file."""
    p = Path(path)
    raw = p.read_bytes()
    if len(raw) < HEADER_SIZE:
        raise ValueError(f"{p}: truncated header ({len(raw)} bytes)")
    magic, version, tap_count, frame_count, dt_s, fc_meas_hz, max_delay_s, \
        reserved = struct.unpack(HEADER_FMT, raw[:HEADER_SIZE])
    if magic != MAGIC:
        raise ValueError(f"{p}: bad magic {magic!r}")
    if version != VERSION:
        raise ValueError(f"{p}: unsupported version {version}")
    if not 1 <= tap_count <= MAX_TAPS:
        raise ValueError(f"{p}: tap_count {tap_count} out of range")
    if frame_count < 2:
        raise ValueError(f"{p}: frame_count {frame_count} < 2")
    if not dt_s > 0.0:
        raise ValueError(f"{p}: dt_s {dt_s} <= 0")
    if reserved != b"\x00" * 24:
        raise ValueError(f"{p}: nonzero reserved header bytes")

    expected = HEADER_SIZE + frame_count * tap_count * RECORD_DTYPE.itemsize
    if len(raw) != expected:
        raise ValueError(f"{p}: size {len(raw)} != expected {expected}")

    records = np.frombuffer(raw[HEADER_SIZE:], dtype=RECORD_DTYPE)
    delays = records["delay_s"].reshape(frame_count, tap_count).copy()
    amplitudes = records["amplitude"].reshape(frame_count, tap_count).copy()

    if not np.all(np.isfinite(delays)) or np.any(delays < 0.0):
        raise ValueError(f"{p}: invalid delay samples")
    if not np.all(np.isfinite(amplitudes)) or np.any(amplitudes < 0.0):
        raise ValueError(f"{p}: invalid amplitude samples")
    if abs(float(delays.max()) - max_delay_s) > 1e-6:
        raise ValueError(f"{p}: header max_delay_s {max_delay_s} != data max "
                         f"{delays.max()}")

    return TapTrajectoryData(dt_s=dt_s, fc_meas_hz=fc_meas_hz,
                             delays=delays, amplitudes=amplitudes)


def catmull_rom_uniform(values: np.ndarray, dt_s: float, t_s: float) -> float:
    """Sample a per-tap track at time ``t_s``.

    Reference for ``TapTrajectory::sample`` in C++: Catmull-Rom on the
    uniform frame grid, endpoints clamped (first/last frames duplicated),
    ``t_s`` clamped to ``[0, (n-1)*dt_s]``. Same Horner form as
    ``SourceDelayLine::read_at``.
    """
    values = np.asarray(values, dtype=np.float64)
    n = len(values)
    last = n - 1
    u = min(max(t_s / dt_s, 0.0), float(last))
    i = min(int(np.floor(u)), last - 1)
    mu = u - i

    p0 = values[max(i - 1, 0)]
    p1 = values[i]
    p2 = values[i + 1]
    p3 = values[min(i + 2, last)]

    return float(p1 + 0.5 * mu * ((p2 - p0)
                 + mu * ((2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3)
                         + mu * (3.0 * (p1 - p2) + p3 - p0))))
