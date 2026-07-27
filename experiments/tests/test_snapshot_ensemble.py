"""Window-allocation + ensemble-build tests for quasi-static snapshots."""
from __future__ import annotations

import json

import numpy as np
import pytest

from lib.snapshot_ensemble import allocate_windows, build_ensemble
from lib.tap_trajectory import read_octt

from .test_quasi_static import write_synthetic_mat
from .test_watermark_converter import synth_tvir


# ---------------------------------------------------------------------------
# Pure allocation logic
# ---------------------------------------------------------------------------

def test_more_files_than_snapshots_uses_whole_files():
    # KAU1-like: 16 files x 33 s, 10 snapshots -> one whole-file window
    # each, spread across the file list.
    specs = allocate_windows([33.0] * 16, 10)
    assert len(specs) == 10
    indices = [s.file_index for s in specs]
    assert len(set(indices)) == 10          # independent soundings
    assert indices == sorted(indices)
    assert indices[0] == 0 and indices[-1] >= 13   # spread, not first-10
    for s in specs:
        assert s.start_s == 0.0
        assert s.duration_s == pytest.approx(33.0)


def test_fewer_files_than_snapshots_splits_non_overlapping():
    # BCH1-like: 4 files x 59.5 s, 10 snapshots -> 3/3/2/2 windows.
    specs = allocate_windows([59.5] * 4, 10)
    assert len(specs) == 10
    per_file: dict[int, list] = {}
    for s in specs:
        per_file.setdefault(s.file_index, []).append(s)
    assert sorted(len(v) for v in per_file.values()) == [2, 2, 3, 3]
    for windows in per_file.values():
        windows.sort(key=lambda s: s.start_s)
        for a, b in zip(windows, windows[1:]):
            assert a.start_s + a.duration_s <= b.start_s + 1e-9
        for w in windows:
            assert w.start_s + w.duration_s <= 59.5 + 1e-9


def test_explicit_window_duration_is_respected():
    specs = allocate_windows([59.5] * 4, 8, window_duration_s=10.0)
    assert len(specs) == 8
    for s in specs:
        assert s.duration_s == pytest.approx(10.0)


def test_window_duration_longer_than_slot_raises():
    with pytest.raises(ValueError, match="window"):
        allocate_windows([30.0] * 2, 4, window_duration_s=20.0)


def test_zero_snapshots_raises():
    with pytest.raises(ValueError):
        allocate_windows([33.0] * 4, 0)


# ---------------------------------------------------------------------------
# Ensemble build on synthetic .mat files
# ---------------------------------------------------------------------------

@pytest.fixture
def synthetic_dataset(tmp_path):
    d = tmp_path / "SYN1" / "mat"
    d.mkdir(parents=True)
    a = synth_tvir([{"tau": lambda t: 0.005, "amp": lambda t: 1.0}])
    a.v0 = -1.0
    b = synth_tvir([{"tau": lambda t: 0.008, "amp": lambda t: 1.0},
                    {"tau": lambda t: 0.012, "amp": lambda t: 0.5}])
    b.v0 = -1.4
    write_synthetic_mat(a, d / "SYN1_001.mat")
    write_synthetic_mat(b, d / "SYN1_002.mat")
    return d


def test_build_ensemble_end_to_end(synthetic_dataset, tmp_path):
    out = tmp_path / "ensemble"
    manifest = build_ensemble(synthetic_dataset, out, n_snapshots=2,
                              record_duration_s=15.0, record_dt_s=0.5)

    assert manifest["dataset"] == "SYN1"
    assert manifest["mean_v0_m_s"] == pytest.approx(-1.2)
    snaps = manifest["snapshots"]
    assert len(snaps) == 2
    assert {s["source_file"].split("/")[-1] for s in snaps} == {
        "SYN1_001.mat", "SYN1_002.mat"}

    for s in snaps:
        octt = out / s["octt_file"]
        data = read_octt(octt)
        assert data.frame_count == 31
        assert len(s["taps"]) == data.tap_count
        for tap in s["taps"]:
            assert set(tap) == {"delay_s", "amplitude", "doppler_hz"}

    # Manifest on disk matches the returned dict.
    on_disk = json.loads((out / "manifest.json").read_text())
    assert on_disk == manifest

    # The two-path file produced a two-tap snapshot.
    tap_counts = sorted(len(s["taps"]) for s in snaps)
    assert tap_counts == [1, 2]


def test_build_ensemble_split_windows(synthetic_dataset, tmp_path):
    # 4 snapshots from 2 files -> 2 non-overlapping windows per file.
    out = tmp_path / "ensemble4"
    manifest = build_ensemble(synthetic_dataset, out, n_snapshots=4)
    snaps = manifest["snapshots"]
    assert len(snaps) == 4
    starts = sorted(s["window_start_s"] for s in snaps
                    if s["source_file"].endswith("SYN1_001.mat"))
    assert len(starts) == 2
    assert starts[1] >= starts[0] + snaps[0]["window_duration_s"] - 1e-9


def test_build_ensemble_rejects_structureless_windows(tmp_path):
    # File layout [good, noise, good]: allocation for n=2 picks files 0
    # and 1; the noise window must be rejected on dynamic range and
    # replaced by a window from the unused good file 2.
    d = tmp_path / "SYN2" / "mat"
    d.mkdir(parents=True)
    good_a = synth_tvir([{"tau": lambda t: 0.005, "amp": lambda t: 1.0}])
    good_b = synth_tvir([{"tau": lambda t: 0.009, "amp": lambda t: 1.0}])
    rng = np.random.default_rng(7)
    from lib.watermark import TVIRData
    noise = TVIRData(
        h=(rng.standard_normal((120, 512))
           + 1j * rng.standard_normal((120, 512))),
        fs_t=good_a.fs_t, fs_tau=good_a.fs_tau, fc=good_a.fc)
    write_synthetic_mat(good_a, d / "SYN2_001.mat")
    write_synthetic_mat(noise, d / "SYN2_002.mat")
    write_synthetic_mat(good_b, d / "SYN2_003.mat")

    out = tmp_path / "ensemble_guard"
    manifest = build_ensemble(d, out, n_snapshots=2)

    snaps = manifest["snapshots"]
    assert len(snaps) == 2
    used = {s["source_file"].split("/")[-1] for s in snaps}
    assert used == {"SYN2_001.mat", "SYN2_003.mat"}
    for s in snaps:
        assert s["dynamic_range_db"] > 20.0
    rejected = manifest["rejected"]
    assert len(rejected) == 1
    assert rejected[0]["source_file"].endswith("SYN2_002.mat")
    assert rejected[0]["dynamic_range_db"] < 20.0


def test_build_ensemble_cli(synthetic_dataset, tmp_path, capsys):
    from experiments.build_snapshot_ensemble import main
    out = tmp_path / "cli_out"
    rc = main([str(synthetic_dataset), str(out), "--n-snapshots", "2"])
    assert rc == 0
    assert (out / "manifest.json").is_file()
    printed = capsys.readouterr().out
    assert "SYN1" in printed
