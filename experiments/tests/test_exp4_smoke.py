"""End-to-end smoke test for the Exp 4 analysis pipeline.

Forward-models the replayed channel in Python: each fake RX session is
the TX chirp delayed by the ground-truth trajectory taps at a known
record time. Exercises the real ``measure_chirp`` / ``analyse_cell`` /
``check_measurements`` code paths without hardware or a built openCREST.
"""
from __future__ import annotations

import struct
import wave
from pathlib import Path

import numpy as np
import pytest

from experiments.exp4_replay_validation import (
    AMP1_REL_DB,
    AMP2_REL_DB,
    FS_HZ,
    SPACING1_AT,
    SPACING2_AT,
    analyse_cell,
    check_measurements,
    make_validation_trajectory,
    render_residual_plot,
    write_residuals_csv,
)

FS = int(FS_HZ)
BASE_DELAY_S = 0.010
CHIRP_TIMES_S = [2.0, 10.0, 20.0, 33.0, 45.0]   # last one: tap2 faded out


def _write_wav(path: Path, samples: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    peak = float(np.max(np.abs(samples))) or 1.0
    pcm = np.clip(samples / peak * 30000.0, -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(pcm.tobytes())


def _chirp(n: int = FS // 50) -> np.ndarray:
    t = np.arange(n) / FS
    f0, f1 = 25e3, 35e3
    return np.sin(2 * np.pi * (f0 * t + 0.5 * (f1 - f0) / t[-1] * t * t))


def _fractional_shift(x: np.ndarray, shift_samples: float,
                      out_len: int) -> np.ndarray:
    idx = np.arange(out_len) - shift_samples
    return np.interp(idx, np.arange(len(x)), x, left=0.0, right=0.0)


@pytest.fixture()
def fake_cell(tmp_path: Path) -> Path:
    cell = tmp_path / "cell_000"
    cell.mkdir()
    make_validation_trajectory(tmp_path / "trajectory.octt")

    tx = _chirp()
    _write_wav(cell / "OA-2-1_tx.wav", np.concatenate(
        [np.zeros(1000), tx, np.zeros(1000)]))

    a1 = 10.0 ** (AMP1_REL_DB / 20.0)
    a2 = 10.0 ** (AMP2_REL_DB / 20.0)
    out_len = len(tx) + int(0.040 * FS)
    for i, t_rec in enumerate(CHIRP_TIMES_S):
        base = BASE_DELAY_S * FS
        rx = _fractional_shift(tx, base, out_len)
        rx += a1 * _fractional_shift(
            tx, base + SPACING1_AT(t_rec) * FS, out_len)
        if t_rec < 40.0:
            rx += a2 * _fractional_shift(
                tx, base + SPACING2_AT(t_rec) * FS, out_len)
        _write_wav(cell / f"OA-2-2_rx_{i:03d}.wav",
                   np.concatenate([np.zeros(2000), rx]))
    return cell


def test_analysis_recovers_trajectory(fake_cell: Path):
    ms = analyse_cell(fake_cell, min_rx_bytes=1000)
    assert len(ms) == len(CHIRP_TIMES_S)

    # Record times inferred from tap-1 spacing track the truth.
    for m, t_true in zip(ms, CHIRP_TIMES_S):
        assert m.record_time_s == pytest.approx(t_true, abs=0.5)

    # Tap 2 fades out after 40 s: measured as nan on the last chirp only.
    assert all(not np.isnan(m.spacing2_samples) for m in ms[:-1])
    assert np.isnan(ms[-1].spacing2_samples)

    # The full assertion set passes on clean synthetic data.
    assert check_measurements(ms) == []


def test_check_flags_bad_spacing(fake_cell: Path):
    ms = analyse_cell(fake_cell, min_rx_bytes=1000)
    ms[1].spacing2_residual = 5.0
    failures = check_measurements(ms)
    assert any("spacing residual" in f for f in failures)


def test_check_flags_non_monotonic_record_time(fake_cell: Path):
    ms = analyse_cell(fake_cell, min_rx_bytes=1000)
    ms[2].record_time_s = ms[0].record_time_s - 1.0
    failures = check_measurements(ms)
    assert any("strictly increasing" in f for f in failures)


def test_outputs_render(fake_cell: Path, tmp_path: Path):
    ms = analyse_cell(fake_cell, min_rx_bytes=1000)
    write_residuals_csv(ms, tmp_path / "residuals.csv")
    render_residual_plot(ms, tmp_path / "fig.pdf")
    assert (tmp_path / "residuals.csv").is_file()
    assert (tmp_path / "fig.pdf").is_file()
    header = (tmp_path / "residuals.csv").read_text().splitlines()[0]
    assert "spacing2_residual_samples" in header


def test_empty_cell_reports_failure(tmp_path: Path):
    empty = tmp_path / "empty_cell"
    empty.mkdir()
    assert check_measurements(analyse_cell(empty)) == ["no chirp measurements"]
