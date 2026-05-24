"""End-to-end smoke test for the Exp 1 analysis pipeline.

Synthesises a fake cell directory by forward-modelling the geometric
scene in Python, then exercises the real ``analyse_cell``,
``write_residuals_csv`` and ``render_waterfall`` code paths. Catches
analysis-pipeline drift (delay-sample convention flips, lag-axis
misalignment) without needing real hardware or a built openCREST.
"""
from __future__ import annotations

import struct
import wave
from pathlib import Path

import numpy as np
import pytest

from experiments.exp1_channel_validation import (
    DEFAULT_FS_HZ,
    DEFAULT_MAX_DELAY_S,
    RECEIVER_DEPTH_M,
    SOUND_SPEED_M_S,
    SOURCE_DEPTH_M,
    WATER_DEPTH_M,
    analyse_cell,
    render_waterfall,
    write_residuals_csv,
)
from experiments.lib.analytical_taps import analytical_taps


FS = DEFAULT_FS_HZ


def _write_wav(path: Path, samples: np.ndarray, fs: int = int(FS)) -> None:
    """Write ``samples`` as mono 16-bit PCM (openCREST's stream_logger
    format)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    peak = float(np.max(np.abs(samples))) or 1.0
    pcm = np.clip(samples / peak * 32767.0, -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(pcm.tobytes())


def _lfm(length: int, f0: float, f1: float, fs: float) -> np.ndarray:
    t = np.arange(length) / fs
    k = (f1 - f0) / (length / fs)
    return np.cos(2.0 * np.pi * (f0 * t + 0.5 * k * t * t))


def _synthesize_cell(cell_dir: Path,
                     range_m: float,
                     *,
                     chirp_len: int = 2048,
                     gamma_surface: float = -0.9,
                     gamma_bottom: float = 0.7,
                     ) -> None:
    """Build a synthetic (modem_a_tx.wav, modem_b_rx_*.wav) pair for the
    given range, using the same method-of-images math as the channel.

    TX: short LFM chirp at 25 kHz +/- 5 kHz. RX: sum of three
    scaled+delayed copies (direct / surface / bottom) with the direct
    path at delay 0 in the RX (matches the IR estimator's
    positive-lag convention). Path-loss amplitudes are skipped because
    the IR estimator picks peak locations only.
    """
    cell_dir.mkdir(parents=True, exist_ok=True)
    tx = _lfm(chirp_len, 20_000.0, 30_000.0, FS)

    taps = analytical_taps(
        range_m,
        water_depth_m   = WATER_DEPTH_M,
        source_depth_m  = SOURCE_DEPTH_M,
        receiver_depth_m= RECEIVER_DEPTH_M,
        sound_speed_m_s = SOUND_SPEED_M_S,
        fs_hz           = FS,
    )
    rel_amps = {
        "direct":  1.0,
        "surface": gamma_surface,
        "bottom":  gamma_bottom,
    }

    # Make the RX long enough to fit chirp + worst-case excess + slack.
    max_excess = int(round(max(t.excess_delay_samples for t in taps)))
    rx_len = chirp_len + max_excess + 1024
    rx = np.zeros(rx_len, dtype=np.float64)
    for t in taps:
        d = int(round(t.excess_delay_samples))
        rx[d:d + chirp_len] += rel_amps[t.name] * tx

    # WAVs are named by USB serial (matches the simulator). The driver
    # picks the largest non-empty RX session, so mirror that: rx_001 is a
    # tiny stub, rx_002 carries the chirp.
    _write_wav(cell_dir / "OA-2-1_tx.wav",       tx)
    _write_wav(cell_dir / "OA-2-2_rx_001.wav",   np.zeros(64, dtype=np.float64))
    _write_wav(cell_dir / "OA-2-2_rx_002.wav",   rx)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_direct_residual_is_zero(tmp_path: Path) -> None:
    cell_dir = tmp_path / "cell0"
    _synthesize_cell(cell_dir, range_m=500.0)
    a = analyse_cell(cell_dir, range_m=500.0, seed=0)
    assert a is not None
    assert a.measured["direct"] == 0.0
    assert a.analytical["direct"] == 0.0


def test_surface_and_bottom_residuals_within_five_samples(tmp_path: Path) -> None:
    """With a synthetic channel and no noise the residual is dominated by
    integer rounding of the analytical delay (well under 1 sample); the
    5-sample slack matches the spec threshold."""
    cell_dir = tmp_path / "cell0"
    _synthesize_cell(cell_dir, range_m=500.0)
    a = analyse_cell(cell_dir, range_m=500.0, seed=0)
    assert a is not None
    surface_resid = abs(a.measured["surface"] - a.analytical["surface"])
    bottom_resid  = abs(a.measured["bottom"]  - a.analytical["bottom"])
    assert surface_resid < 5.0, f"surface residual {surface_resid}"
    assert bottom_resid  < 5.0, f"bottom residual {bottom_resid}"


@pytest.mark.parametrize("range_m", [200.0, 500.0, 1000.0])
def test_residuals_under_5_samples_across_sweep_extremes(
        tmp_path: Path, range_m: float) -> None:
    """Same residual threshold exercised at both ends of the default range
    sweep to catch range-dependent windowing bugs."""
    cell_dir = tmp_path / f"cell_R{int(range_m)}"
    _synthesize_cell(cell_dir, range_m=range_m)
    a = analyse_cell(cell_dir, range_m=range_m, seed=0)
    assert a is not None
    for name in ("direct", "surface", "bottom"):
        resid = abs(a.measured[name] - a.analytical[name])
        assert resid < 5.0, f"R={range_m} {name} residual {resid}"


def test_missing_rx_returns_none(tmp_path: Path) -> None:
    cell = tmp_path / "empty"
    cell.mkdir()
    # No WAVs.
    assert analyse_cell(cell, range_m=500.0, seed=0) is None


def test_residuals_csv_has_one_row_per_path(tmp_path: Path) -> None:
    cell_dir = tmp_path / "cell0"
    _synthesize_cell(cell_dir, range_m=500.0)
    a = analyse_cell(cell_dir, range_m=500.0, seed=0)
    assert a is not None

    csv_path = tmp_path / "residuals.csv"
    write_residuals_csv([a], csv_path)
    rows = [r for r in csv_path.read_text().splitlines() if r]
    # Header + 3 paths.
    assert len(rows) == 4
    assert rows[0].startswith("range_m,seed,path,")


def test_discover_cells_reads_range_and_seed_from_yaml(tmp_path: Path) -> None:
    """The analysis loop walks the output directory and reads each cell's
    scenario.yaml so it doesn't depend on the runner's param hash."""
    from experiments.exp1_channel_validation import _discover_cells

    # Two well-formed cells + one stray directory with no scenario.
    for cell_id, R, seed in (("abc111", 200.0, 0), ("def222", 700.5, 7)):
        d = tmp_path / cell_id
        d.mkdir()
        (d / "scenario.yaml").write_text(
            f"name: x\n"
            f"random_seed: {seed}\n"
            f"channels:\n"
            f"  - from: modem_a\n"
            f"    to: modem_b\n"
            f"    range_m: {R}\n"
            f"    initial_range_m: {R}\n"
        )
    (tmp_path / "stray").mkdir()
    (tmp_path / "stray" / "other.txt").write_text("nope")

    found = _discover_cells(tmp_path)
    assert sorted((cd.name, R, seed) for cd, R, seed in found) == [
        ("abc111", 200.0, 0), ("def222", 700.5, 7),
    ]


def test_render_waterfall_writes_pdf(tmp_path: Path) -> None:
    analyses = []
    for R in (300.0, 500.0, 700.0):
        cell_dir = tmp_path / f"R{int(R)}"
        _synthesize_cell(cell_dir, range_m=R)
        a = analyse_cell(cell_dir, range_m=R, seed=0)
        assert a is not None
        analyses.append(a)

    pdf_path = tmp_path / "fig_validation.pdf"
    render_waterfall(analyses, pdf_path)
    assert pdf_path.is_file()
    assert pdf_path.stat().st_size > 1000
