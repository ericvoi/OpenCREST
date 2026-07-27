"""Synthetic-TVIR ground-truth tests for the Watermark converter."""
from __future__ import annotations

import numpy as np
import pytest

from lib.tap_trajectory import read_octt, write_octt
from lib.watermark import (GUARD_DELAY_S, TVIRData, convert, extract_tracks,
                           tracks_to_trajectories)

FS_T = 40.0          # snapshot rate, Hz
FS_TAU = 16_000.0    # delay-axis rate, Hz
FC = 14_000.0        # measurement carrier, Hz
FRAMES = 120         # 3 s record
BINS = 512           # 32 ms delay coverage


def synth_tvir(paths, *, frames=FRAMES, v0=0.0):
    """Build a band-limited synthetic TVIR.

    ``paths`` is a list of dicts with keys:
      tau: callable t -> delay seconds
      amp: callable t -> linear amplitude (0 = path absent)
      phase: optional callable t -> extra phase (rad) beyond -2*pi*fc*tau
    """
    t = np.arange(frames) / FS_T
    bins = np.arange(BINS)
    h = np.zeros((frames, BINS), dtype=np.complex128)
    for p in paths:
        for i, ti in enumerate(t):
            a = p["amp"](ti)
            if a <= 0.0:
                continue
            tau = p["tau"](ti)
            extra = p["phase"](ti) if "phase" in p else 0.0
            gain = a * np.exp(1j * (-2.0 * np.pi * FC * tau + extra))
            h[i] += gain * np.sinc(bins - tau * FS_TAU)
    return TVIRData(h=h, fs_t=FS_T, fs_tau=FS_TAU, fc=FC, v0=v0)


def two_constant_paths():
    return synth_tvir([
        {"tau": lambda t: 0.005, "amp": lambda t: 1.0},
        {"tau": lambda t: 0.010, "amp": lambda t: 0.5},
    ])


def test_extracts_two_constant_paths():
    tracks = extract_tracks(two_constant_paths())
    assert len(tracks) == 2
    tracks.sort(key=lambda tr: tr.delays_s[0])
    bin_s = 1.0 / FS_TAU
    assert tracks[0].delays_s[0] == pytest.approx(0.005, abs=0.2 * bin_s)
    assert tracks[1].delays_s[0] == pytest.approx(0.010, abs=0.2 * bin_s)
    assert len(tracks[0].delays_s) == FRAMES
    assert len(tracks[1].delays_s) == FRAMES


def test_relative_amplitudes_within_half_db():
    result = convert(two_constant_paths())
    assert result.amplitudes.shape[1] == 2
    # Sort taps by delay; ratio of mid-record amplitudes.
    order = np.argsort(result.delays[FRAMES // 2])
    a = result.amplitudes[FRAMES // 2][order]
    ratio_db = 20.0 * np.log10(a[1] / a[0])
    assert ratio_db == pytest.approx(-6.02, abs=0.5)
    # Peak normalisation: global max is exactly 1.
    assert result.amplitudes.max() == pytest.approx(1.0)


def test_recovers_delay_rate_within_5_percent():
    slope = 1e-4     # s/s, receding path
    tvir = synth_tvir([
        {"tau": lambda t: 0.005 + slope * t, "amp": lambda t: 1.0},
    ])
    result = convert(tvir)
    t = np.arange(FRAMES) / FS_T
    # Ignore the smoothed edges.
    body = slice(10, FRAMES - 10)
    fit = np.polyfit(t[body], result.delays[body, 0], 1)
    assert fit[0] == pytest.approx(slope, rel=0.05)


def test_reinstalls_mean_doppler_ramp():
    v0 = 1.0    # m/s
    flat = synth_tvir([{"tau": lambda t: 0.005, "amp": lambda t: 1.0}])
    with_v0 = synth_tvir([{"tau": lambda t: 0.005, "amp": lambda t: 1.0}],
                         v0=v0)
    r_flat = convert(flat)
    r_v0 = convert(with_v0)
    t = np.arange(FRAMES) / FS_T
    body = slice(10, FRAMES - 10)
    slope_flat = np.polyfit(t[body], r_flat.delays[body, 0], 1)[0]
    slope_v0 = np.polyfit(t[body], r_v0.delays[body, 0], 1)[0]
    assert slope_v0 - slope_flat == pytest.approx(v0 / 1500.0, rel=0.05)


def test_birth_produces_zero_amplitude_head_and_held_delay():
    birth_t = 30 / FS_T
    tvir = synth_tvir([
        {"tau": lambda t: 0.005, "amp": lambda t: 1.0},
        {"tau": lambda t: 0.012, "amp": lambda t: 0.6 if t >= birth_t else 0.0},
    ])
    result = convert(tvir)
    order = np.argsort(result.delays[FRAMES // 2])
    late = order[1]
    # Amplitude is zero strictly before birth...
    assert np.all(result.amplitudes[:29, late] == 0.0)
    assert result.amplitudes[FRAMES // 2, late] > 0.0
    # ...and the delay holds a finite value there (interpolation-safe).
    head = result.delays[:29, late]
    assert np.all(np.isfinite(head))
    assert np.ptp(head) == pytest.approx(0.0, abs=1e-12)


def test_pi_flip_becomes_half_cycle_delay_step():
    flip_t = 60 / FS_T
    tvir = synth_tvir([
        {"tau": lambda t: 0.005, "amp": lambda t: 1.0,
         "phase": lambda t: np.pi if t >= flip_t else 0.0},
    ], frames=FRAMES)
    result = convert(tvir, smooth_frames=3)
    early = float(np.mean(result.delays[10:50, 0]))
    late = float(np.mean(result.delays[70:110, 0]))
    half_cycle = 0.5 / FC
    assert abs(late - early) == pytest.approx(half_cycle, rel=0.2)


def test_max_taps_keeps_strongest_and_reports_capture():
    tvir = two_constant_paths()
    tracks = extract_tracks(tvir)
    result = tracks_to_trajectories(tracks, tvir, max_taps=1)
    assert result.delays.shape[1] == 1
    # Kept the stronger (unit-amplitude) path: its own peak is the global
    # peak, so normalisation leaves it at exactly 1.0.
    assert result.amplitudes.max() == pytest.approx(1.0)
    # Energy split 1 : 0.25 -> captured 0.8.
    assert result.captured_energy_fraction == pytest.approx(0.8, abs=0.02)

    full = tracks_to_trajectories(tracks, tvir)
    assert full.captured_energy_fraction == pytest.approx(1.0)


def test_delays_respect_guard_floor():
    tvir = synth_tvir([
        {"tau": lambda t: 0.001 + 5e-5 * t, "amp": lambda t: 1.0},
    ])
    result = convert(tvir)
    assert result.delays.min() == pytest.approx(GUARD_DELAY_S, abs=1e-9)


def test_anchor_offset_lands_delays_back_on_measured_axis():
    tvir = two_constant_paths()
    result = convert(tvir)
    measured = result.delays - result.delay_anchor_offset_s
    order = np.argsort(measured[FRAMES // 2])
    bin_s = 1.0 / FS_TAU
    assert measured[FRAMES // 2][order][0] == pytest.approx(0.005,
                                                            abs=0.2 * bin_s)
    assert measured[FRAMES // 2][order][1] == pytest.approx(0.010,
                                                            abs=0.2 * bin_s)


def test_qa_figure_renders(tmp_path):
    from lib.watermark import render_conversion_qa
    tvir = two_constant_paths()
    result = convert(tvir)
    out = render_conversion_qa(tvir, result, tmp_path / "qa.pdf")
    assert out.is_file()
    assert out.stat().st_size > 1000


def test_round_trips_through_octt(tmp_path):
    result = convert(two_constant_paths())
    path = tmp_path / "converted.octt"
    write_octt(path, result.dt_s, result.fc_meas_hz,
               result.delays, result.amplitudes)
    data = read_octt(path)
    assert data.tap_count == 2
    assert data.frame_count == FRAMES
    np.testing.assert_allclose(data.delays, result.delays)
