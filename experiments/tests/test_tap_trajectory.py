"""Tests for the .octt tap-trajectory writer/reader."""
import struct

import numpy as np
import pytest

from lib.tap_trajectory import (HEADER_SIZE, MAX_DELAY_S, MAX_TAPS,
                                catmull_rom_uniform, read_octt, write_octt)


def _two_tap(frames=5, taps=2):
    delays = np.linspace(0.001, 0.004, frames)[:, None] * np.arange(1, taps + 1)
    amps = np.linspace(1.0, 0.5, frames)[:, None] * np.ones(taps)
    return delays, amps


def test_round_trip_bit_exact(tmp_path):
    delays, amps = _two_tap()
    path = tmp_path / "t.octt"
    write_octt(path, dt_s=0.025, fc_meas_hz=35e3, delays=delays, amplitudes=amps)
    data = read_octt(path)
    assert data.dt_s == 0.025
    assert data.fc_meas_hz == 35e3
    assert data.tap_count == 2
    assert data.frame_count == 5
    assert data.duration_s == pytest.approx(4 * 0.025)
    np.testing.assert_array_equal(data.delays, delays)
    np.testing.assert_array_equal(data.amplitudes,
                                  np.asarray(amps, dtype=np.float32))


def test_file_size_matches_layout(tmp_path):
    delays, amps = _two_tap(frames=7, taps=3)
    path = tmp_path / "t.octt"
    write_octt(path, 0.05, 14e3, delays, amps)
    assert path.stat().st_size == HEADER_SIZE + 7 * 3 * 16


@pytest.mark.parametrize("mutate,err", [
    (dict(delays_shape=(1, 2)), "frame_count"),
    (dict(taps=0), "tap_count"),
    (dict(taps=MAX_TAPS + 1), "tap_count"),
    (dict(dt_s=0.0), "dt_s"),
    (dict(dt_s=float("nan")), "dt_s"),
    (dict(fc=-1.0), "fc_meas_hz"),
    (dict(delay_value=float("nan")), "non-finite"),
    (dict(delay_value=-1e-9), ">= 0"),
    (dict(amp_value=-0.1), ">= 0"),
    (dict(delay_value=MAX_DELAY_S + 0.001), "exceeds"),
])
def test_writer_rejects_invalid(tmp_path, mutate, err):
    frames = mutate.get("delays_shape", (5, None))[0] if "delays_shape" in mutate else 5
    taps = mutate.get("taps", 2)
    if taps == 0:
        delays = np.zeros((frames, 0))
        amps = np.zeros((frames, 0))
    else:
        delays, amps = _two_tap(frames=frames, taps=taps)
    if "delay_value" in mutate:
        delays[1, 0] = mutate["delay_value"]
    if "amp_value" in mutate:
        amps[1, 0] = mutate["amp_value"]
    with pytest.raises(ValueError, match=err):
        write_octt(tmp_path / "bad.octt",
                   dt_s=mutate.get("dt_s", 0.025),
                   fc_meas_hz=mutate.get("fc", 35e3),
                   delays=delays, amplitudes=amps)


def test_writer_rejects_shape_mismatch(tmp_path):
    delays, amps = _two_tap()
    with pytest.raises(ValueError, match="shape mismatch"):
        write_octt(tmp_path / "bad.octt", 0.025, 35e3, delays, amps[:, :1])


def test_reader_rejects_bad_magic(tmp_path):
    delays, amps = _two_tap()
    path = tmp_path / "t.octt"
    write_octt(path, 0.025, 35e3, delays, amps)
    raw = bytearray(path.read_bytes())
    raw[:4] = b"NOPE"
    path.write_bytes(bytes(raw))
    with pytest.raises(ValueError, match="magic"):
        read_octt(path)


def test_reader_rejects_truncated(tmp_path):
    delays, amps = _two_tap()
    path = tmp_path / "t.octt"
    write_octt(path, 0.025, 35e3, delays, amps)
    path.write_bytes(path.read_bytes()[:-8])
    with pytest.raises(ValueError, match="size"):
        read_octt(path)


def test_reader_rejects_header_max_delay_mismatch(tmp_path):
    delays, amps = _two_tap()
    path = tmp_path / "t.octt"
    write_octt(path, 0.025, 35e3, delays, amps)
    raw = bytearray(path.read_bytes())
    raw[32:40] = struct.pack("<d", 0.123)
    path.write_bytes(bytes(raw))
    with pytest.raises(ValueError, match="max_delay"):
        read_octt(path)


def test_reader_rejects_nonzero_reserved(tmp_path):
    delays, amps = _two_tap()
    path = tmp_path / "t.octt"
    write_octt(path, 0.025, 35e3, delays, amps)
    raw = bytearray(path.read_bytes())
    raw[41] = 7
    path.write_bytes(bytes(raw))
    with pytest.raises(ValueError, match="reserved"):
        read_octt(path)


def test_catmull_rom_hits_frames_exactly():
    values = np.array([1.0, 3.0, 2.0, 5.0])
    for i, v in enumerate(values):
        assert catmull_rom_uniform(values, 0.5, i * 0.5) == pytest.approx(v)


def test_catmull_rom_clamps_time():
    values = np.array([1.0, 3.0, 2.0])
    assert catmull_rom_uniform(values, 0.1, -5.0) == pytest.approx(1.0)
    assert catmull_rom_uniform(values, 0.1, 99.0) == pytest.approx(2.0)


def test_catmull_rom_linear_track_is_exact():
    # Catmull-Rom reproduces linear data exactly (including at endpoints,
    # thanks to the clamped virtual duplicates... which for linear data
    # slightly flatten the end segments' slope; interior must be exact).
    values = np.linspace(0.0, 3.0, 4)
    assert catmull_rom_uniform(values, 1.0, 1.5) == pytest.approx(1.5)
