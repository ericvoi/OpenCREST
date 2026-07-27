"""Synthetic-TVIR ground-truth tests for quasi-static tap extraction.

A quasi-static snapshot freezes one window of a measured TVIR into taps
{delay, amplitude, Doppler}. The chain under test: windowed scattering
function -> 2-D delay-Doppler peak pick -> linear-ramp trajectory grids
(constant Doppler == linear delay ramp d tau/dt = -nu/fc_meas).
"""
from __future__ import annotations

import numpy as np
import pytest

from lib.quasi_static import (QuasiStaticTaps, extract_quasi_static,
                              quasi_static_to_trajectories,
                              render_quasi_static_qa, scattering_function)
from lib.tap_trajectory import catmull_rom_uniform, read_octt, write_octt
from lib.watermark import GUARD_DELAY_S, SOUND_SPEED_M_S, TVIRData

from .test_watermark_converter import FC, FS_T, FS_TAU, synth_tvir

BIN_S = 1.0 / FS_TAU


def two_static_paths():
    # Delays land exactly on delay bins (0.005 * 16k = 80) so the synthetic
    # sinc kernel has no delay-axis leakage.
    return synth_tvir([
        {"tau": lambda t: 0.005, "amp": lambda t: 1.0},
        {"tau": lambda t: 0.010, "amp": lambda t: 0.5},
    ])


# ---------------------------------------------------------------------------
# Scattering function
# ---------------------------------------------------------------------------

def test_scattering_function_peak_position_and_scale():
    sf = scattering_function(two_static_paths())
    i, j = np.unravel_index(np.argmax(sf.amplitude), sf.amplitude.shape)
    # Static path: line at 0 Hz Doppler, 5 ms delay, unit amplitude (the
    # window normalisation makes a constant-amplitude path read ~1.0).
    assert sf.doppler_hz[i] == pytest.approx(0.0, abs=1e-9)
    assert sf.delay_s[j] == pytest.approx(0.005, abs=0.2 * BIN_S)
    assert sf.amplitude[i, j] == pytest.approx(1.0, abs=0.02)
    assert sf.resolution_hz == pytest.approx(FS_T / two_static_paths().h.shape[0])


def test_scattering_function_rejects_tiny_windows():
    tvir = two_static_paths()
    with pytest.raises(ValueError):
        scattering_function(tvir, start_s=0.0, duration_s=4.0 / FS_T)


# ---------------------------------------------------------------------------
# Extraction
# ---------------------------------------------------------------------------

def test_two_static_paths_extracted():
    qs = extract_quasi_static(two_static_paths())
    assert qs.delays_s.shape == (2,)
    order = np.argsort(qs.delays_s)
    assert qs.delays_s[order[0]] == pytest.approx(0.005, abs=0.2 * BIN_S)
    assert qs.delays_s[order[1]] == pytest.approx(0.010, abs=0.2 * BIN_S)
    # Static paths: no residual Doppler.
    np.testing.assert_allclose(qs.doppler_hz, 0.0, atol=0.05)
    # Peak-normalised amplitudes, ratio -6.02 dB.
    assert qs.amplitudes.max() == pytest.approx(1.0)
    ratio_db = 20.0 * np.log10(qs.amplitudes[order[1]] / qs.amplitudes[order[0]])
    assert ratio_db == pytest.approx(-6.02, abs=0.5)
    assert qs.captured_energy_fraction == pytest.approx(1.0, abs=0.02)
    assert qs.fc_meas_hz == FC


def test_pure_doppler_line_recovered():
    nu = 0.7    # Hz, positive rotation = shrinking delay = closing
    tvir = synth_tvir([
        {"tau": lambda t: 0.005, "amp": lambda t: 1.0,
         "phase": lambda t: 2.0 * np.pi * nu * t},
    ])
    qs = extract_quasi_static(tvir, max_taps=1)
    assert qs.doppler_hz[0] == pytest.approx(nu, abs=0.05)
    # Constant-Doppler tap == linear delay ramp with slope -nu/fc.
    traj = quasi_static_to_trajectories(qs, record_duration_s=20.0, dt_s=0.5)
    t = np.arange(traj.delays.shape[0]) * 0.5
    slope = np.polyfit(t, traj.delays[:, 0], 1)[0]
    assert slope == pytest.approx(-nu / FC, rel=0.08)


def test_delay_ramp_maps_to_doppler():
    s = 2e-5       # s/s receding; over 6 s the path drifts ~1 delay bin
    tvir = synth_tvir([
        {"tau": lambda t: 0.005 + s * t, "amp": lambda t: 1.0},
    ], frames=240)
    qs = extract_quasi_static(tvir, max_taps=1)
    assert qs.doppler_hz[0] == pytest.approx(-FC * s, abs=0.03)
    # Round trip: the emitted ramp recovers the original delay rate.
    traj = quasi_static_to_trajectories(qs, record_duration_s=20.0, dt_s=0.5)
    t = np.arange(traj.delays.shape[0]) * 0.5
    slope = np.polyfit(t, traj.delays[:, 0], 1)[0]
    assert slope == pytest.approx(s, rel=0.15)
    # Extracted delay is the window-mean delay of the drifting path.
    mean_tau = 0.005 + s * (240 / FS_T) / 2.0
    assert qs.delays_s[0] == pytest.approx(mean_tau, abs=1.5 * BIN_S)


def test_two_paths_same_delay_different_doppler():
    """The scattering-function advantage: same-delay paths separated by
    Doppler become two distinct taps (impossible from one |h| snapshot)."""
    tvir = synth_tvir([
        {"tau": lambda t: 0.006, "amp": lambda t: 1.0,
         "phase": lambda t: 2.0 * np.pi * 1.0 * t},
        {"tau": lambda t: 0.006, "amp": lambda t: 0.7,
         "phase": lambda t: -2.0 * np.pi * 1.0 * t},
    ])
    qs = extract_quasi_static(tvir)
    assert qs.delays_s.shape == (2,)
    order = np.argsort(qs.doppler_hz)
    assert qs.doppler_hz[order[0]] == pytest.approx(-1.0, abs=0.08)
    assert qs.doppler_hz[order[1]] == pytest.approx(1.0, abs=0.08)
    np.testing.assert_allclose(qs.delays_s, 0.006, atol=0.2 * BIN_S)
    ratio_db = 20.0 * np.log10(qs.amplitudes[order[0]] / qs.amplitudes[order[1]])
    assert ratio_db == pytest.approx(20.0 * np.log10(0.7), abs=0.5)


def test_windowing_isolates_record_segments():
    switch = 1.5
    tvir = synth_tvir([
        {"tau": lambda t: 0.004, "amp": lambda t: 1.0 if t < switch else 0.0},
        {"tau": lambda t: 0.011, "amp": lambda t: 0.0 if t < switch else 1.0},
    ])
    early = extract_quasi_static(tvir, start_s=0.0, duration_s=switch,
                                 max_taps=1)
    late = extract_quasi_static(tvir, start_s=switch, duration_s=switch,
                                max_taps=1)
    assert early.delays_s[0] == pytest.approx(0.004, abs=0.2 * BIN_S)
    assert late.delays_s[0] == pytest.approx(0.011, abs=0.2 * BIN_S)
    assert early.window_start_s == pytest.approx(0.0)
    assert late.window_start_s == pytest.approx(switch, abs=1.0 / FS_T)


def test_dynamic_range_separates_structure_from_noise():
    structured = extract_quasi_static(two_static_paths())
    assert structured.dynamic_range_db > 30.0

    rng = np.random.default_rng(0)
    noise = TVIRData(
        h=(rng.standard_normal((120, 512))
           + 1j * rng.standard_normal((120, 512))),
        fs_t=FS_T, fs_tau=FS_TAU, fc=FC)
    unstructured = extract_quasi_static(noise)
    assert unstructured.dynamic_range_db < 18.0


def test_max_taps_keeps_strongest_and_reports_capture():
    qs = extract_quasi_static(two_static_paths(), max_taps=1)
    assert qs.delays_s.shape == (1,)
    assert qs.delays_s[0] == pytest.approx(0.005, abs=0.2 * BIN_S)
    assert qs.amplitudes[0] == pytest.approx(1.0)
    # Energy split 1 : 0.25 -> captured 0.8.
    assert qs.captured_energy_fraction == pytest.approx(0.8, abs=0.02)


# ---------------------------------------------------------------------------
# Trajectory emission (linear ramps)
# ---------------------------------------------------------------------------

def _one_tap_qs(doppler_hz: float, delay_s: float = 0.005,
                v0: float = 0.0) -> QuasiStaticTaps:
    return QuasiStaticTaps(
        delays_s=np.array([delay_s]),
        doppler_hz=np.array([doppler_hz]),
        amplitudes=np.array([1.0]),
        fc_meas_hz=FC,
        v0_m_s=v0,
        window_start_s=0.0,
        window_duration_s=3.0,
        doppler_resolution_hz=FS_T / 120,
        captured_energy_fraction=1.0,
        normalization_db=0.0,
        dynamic_range_db=40.0,
    )


def test_trajectories_are_exact_linear_ramps():
    qs = _one_tap_qs(doppler_hz=0.7)
    traj = quasi_static_to_trajectories(qs, record_duration_s=30.0, dt_s=1.0)
    frames = traj.delays.shape[0]
    assert frames == 31
    assert traj.dt_s == 1.0
    # Constant amplitude, constant frame-to-frame delay step.
    np.testing.assert_allclose(traj.amplitudes, 1.0)
    steps = np.diff(traj.delays[:, 0])
    np.testing.assert_allclose(steps, -0.7 / FC, rtol=1e-9)
    # Interior Catmull-Rom reproduces the line exactly (this is what the
    # C++ renderer interpolates).
    tau0 = traj.delays[0, 0]
    for t in (1.0, 7.3, 12.34, 28.9):
        got = catmull_rom_uniform(traj.delays[:, 0], traj.dt_s, t)
        assert got == pytest.approx(tau0 - 0.7 / FC * t, abs=1e-15)
    # Anchored onto the guard floor.
    assert traj.delays.min() == pytest.approx(GUARD_DELAY_S, abs=1e-12)


def test_v0_reinstalled_as_common_ramp():
    v0 = 1.0
    with_v0 = quasi_static_to_trajectories(
        _one_tap_qs(0.0, v0=v0), record_duration_s=10.0, dt_s=0.5)
    without = quasi_static_to_trajectories(
        _one_tap_qs(0.0, v0=v0), record_duration_s=10.0, dt_s=0.5,
        reinstall_v0=False)
    t = np.arange(with_v0.delays.shape[0]) * 0.5
    slope_with = np.polyfit(t, with_v0.delays[:, 0], 1)[0]
    slope_without = np.polyfit(t, without.delays[:, 0], 1)[0]
    assert slope_with - slope_without == pytest.approx(
        v0 / SOUND_SPEED_M_S, rel=1e-9)


def test_record_exceeding_delay_cap_raises():
    # Anchoring absorbs any common delay offset, so the 0.2 s cap binds on
    # the delay SPAN plus drift: 0.19 s spread + ~1 m/s recession over a
    # 30 s record crosses it.
    qs = QuasiStaticTaps(
        delays_s=np.array([0.0, 0.19]),
        doppler_hz=np.array([0.0, -FC * 6.7e-4]),
        amplitudes=np.array([1.0, 0.5]),
        fc_meas_hz=FC, v0_m_s=0.0,
        window_start_s=0.0, window_duration_s=3.0,
        doppler_resolution_hz=FS_T / 120,
        captured_energy_fraction=1.0, normalization_db=0.0,
    )
    with pytest.raises(ValueError, match="record"):
        quasi_static_to_trajectories(qs, record_duration_s=30.0, dt_s=1.0)


def test_round_trips_through_octt(tmp_path):
    qs = extract_quasi_static(two_static_paths())
    traj = quasi_static_to_trajectories(qs, record_duration_s=15.0, dt_s=0.5)
    path = tmp_path / "quasi.octt"
    write_octt(path, traj.dt_s, traj.fc_meas_hz, traj.delays, traj.amplitudes)
    data = read_octt(path)
    assert data.tap_count == 2
    assert data.frame_count == traj.delays.shape[0]
    np.testing.assert_allclose(data.delays, traj.delays)


def test_qa_figure_renders(tmp_path):
    tvir = two_static_paths()
    qs = extract_quasi_static(tvir)
    out = render_quasi_static_qa(tvir, qs, tmp_path / "qa.pdf")
    assert out.is_file()
    assert out.stat().st_size > 1000


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def write_synthetic_mat(tvir: TVIRData, path) -> None:
    from scipy.io import savemat
    savemat(str(path), {"h": tvir.h, "fs_t": tvir.fs_t,
                        "fs_tau": tvir.fs_tau, "fc": tvir.fc,
                        "V0": tvir.v0})


def test_cli_quasi_static_end_to_end(tmp_path, capsys):
    from experiments.convert_watermark import main

    mat = tmp_path / "synthetic.mat"
    write_synthetic_mat(two_static_paths(), mat)
    out = tmp_path / "snap.octt"
    rc = main([str(mat), str(out), "--quasi-static", "--plot",
               "--record-duration-s", "15", "--record-dt-s", "0.5"])
    assert rc == 0
    data = read_octt(out)
    assert data.tap_count == 2
    assert data.frame_count == 31
    assert (tmp_path / "snap.octt.qa.pdf").is_file()
    printed = capsys.readouterr().out
    assert "quasi-static taps" in printed
    assert "Doppler" in printed
