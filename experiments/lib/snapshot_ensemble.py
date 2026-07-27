"""Build a quasi-static snapshot ensemble from a Watermark dataset.

One BER-vs-SNR datapoint pools runs over ~10 quasi-static snapshots, so
the ensemble — not repeat runs — carries the channel-to-channel
variance. Snapshots come from independent sounding files when the
dataset ships enough of them; otherwise each file is split into
non-overlapping windows.

Output: one ``.octt`` per snapshot plus ``manifest.json`` describing
where each snapshot came from and what it contains (per-tap
delay/amplitude/Doppler, measured v0, captured energy). The experiment
harness and the geometric-twin generator both read the manifest, so the
twin's radial velocity can use the *measured* mean Doppler instead of a
documentation guess.
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from .quasi_static import (extract_quasi_static,
                           quasi_static_to_trajectories,
                           render_quasi_static_qa)
from .tap_trajectory import MAX_TAPS, write_octt
from .watermark import load_tvir


@dataclass(frozen=True)
class WindowSpec:
    """One snapshot window inside one sounding file."""
    file_index: int
    start_s: float
    duration_s: float


def allocate_windows(file_durations_s: Sequence[float],
                     n_snapshots: int,
                     *,
                     window_duration_s: float | None = None,
                     ) -> list[WindowSpec]:
    """Spread ``n_snapshots`` windows over the dataset's files.

    With at least as many files as snapshots, each window is a distinct
    file (evenly spread across the list) — independent channel draws.
    With fewer files, each file gets an equal share of non-overlapping
    slots. ``window_duration_s`` caps the window length inside its slot;
    by default a window fills its whole slot.
    """
    n_files = len(file_durations_s)
    if n_snapshots < 1:
        raise ValueError(f"n_snapshots must be >= 1, got {n_snapshots}")
    if n_files == 0:
        raise ValueError("no sounding files to allocate windows over")

    specs: list[WindowSpec] = []
    if n_snapshots <= n_files:
        for k in range(n_snapshots):
            i = k * n_files // n_snapshots
            duration = file_durations_s[i]
            if window_duration_s is not None:
                if window_duration_s > duration + 1e-9:
                    raise ValueError(
                        f"window {window_duration_s} s exceeds file "
                        f"{i}'s {duration:.1f} s duration")
                duration = window_duration_s
            specs.append(WindowSpec(i, 0.0, duration))
        return specs

    base, extra = divmod(n_snapshots, n_files)
    for i, file_duration in enumerate(file_durations_s):
        count = base + (1 if i < extra else 0)
        slot = file_duration / count
        duration = slot
        if window_duration_s is not None:
            if window_duration_s > slot + 1e-9:
                raise ValueError(
                    f"window {window_duration_s} s exceeds the "
                    f"{slot:.1f} s slot ({count} windows in file {i}'s "
                    f"{file_duration:.1f} s)")
            duration = window_duration_s
        for k in range(count):
            specs.append(WindowSpec(i, k * slot, duration))
    return specs


def dataset_label(files: Sequence[Path]) -> str:
    """Dataset name from the sounding filenames (``BCH1_001.mat`` ->
    ``BCH1``); falls back to the first file's stem."""
    stem = files[0].stem
    return stem.split("_")[0] if "_" in stem else stem


def build_ensemble(dataset_dir: str | Path,
                   out_dir: str | Path,
                   *,
                   n_snapshots: int = 10,
                   window_duration_s: float | None = None,
                   max_taps: int = MAX_TAPS,
                   prominence_db: float = -30.0,
                   record_duration_s: float = 30.0,
                   record_dt_s: float = 1.0,
                   min_dynamic_range_db: float = 20.0,
                   qa_plots: bool = False,
                   pattern: str = "*.mat",
                   ) -> dict:
    """Extract ``n_snapshots`` quasi-static tap sets from the dataset and
    write one ``.octt`` each plus ``manifest.json`` under ``out_dir``.

    Windows whose scattering function is structureless (dynamic range
    below ``min_dynamic_range_db`` — noise, sounding not started) are
    rejected and replaced by windows from files the allocation didn't
    use; when no spares remain the ensemble comes up short with a
    warning. Rejections are recorded under ``manifest["rejected"]``.

    Returns the manifest dict.
    """
    dataset_dir = Path(dataset_dir)
    out = Path(out_dir)
    files = sorted(dataset_dir.glob(pattern))
    if not files:
        raise ValueError(f"no {pattern} files in {dataset_dir}")

    tvirs = {}

    def tvir_for(i: int):
        if i not in tvirs:
            tvirs[i] = load_tvir(files[i])
        return tvirs[i]

    durations = []
    for i in range(len(files)):
        tv = tvir_for(i)
        durations.append(tv.h.shape[0] / tv.fs_t)

    specs = list(allocate_windows(durations, n_snapshots,
                                  window_duration_s=window_duration_s))
    spare_files = [i for i in range(len(files))
                   if i not in {s.file_index for s in specs}]
    label = dataset_label(files)
    out.mkdir(parents=True, exist_ok=True)

    snapshots = []
    rejected = []
    queue = list(specs)
    while queue and len(snapshots) < n_snapshots:
        spec = queue.pop(0)
        tvir = tvir_for(spec.file_index)
        taps = extract_quasi_static(
            tvir,
            start_s=spec.start_s,
            duration_s=spec.duration_s,
            max_taps=max_taps,
            prominence_db=prominence_db,
        )
        if taps.dynamic_range_db < min_dynamic_range_db:
            rejected.append({
                "source_file": str(files[spec.file_index]),
                "window_start_s": taps.window_start_s,
                "window_duration_s": taps.window_duration_s,
                "dynamic_range_db": taps.dynamic_range_db,
            })
            sys.stderr.write(
                f"[ensemble] {files[spec.file_index].name} "
                f"[{taps.window_start_s:.1f}, "
                f"{taps.window_start_s + taps.window_duration_s:.1f}] s: "
                f"scattering function has only "
                f"{taps.dynamic_range_db:.1f} dB dynamic range "
                f"(< {min_dynamic_range_db:.0f}) — no channel structure; "
                "window rejected.\n")
            if spare_files:
                i = spare_files.pop(0)
                duration = durations[i]
                if window_duration_s is not None:
                    duration = min(window_duration_s, duration)
                queue.append(WindowSpec(i, 0.0, duration))
            continue

        traj = quasi_static_to_trajectories(
            taps,
            record_duration_s=record_duration_s,
            dt_s=record_dt_s,
        )
        idx = len(snapshots)
        octt_name = f"{label}_snap{idx:02d}.octt"
        write_octt(out / octt_name, traj.dt_s, traj.fc_meas_hz,
                   traj.delays, traj.amplitudes)
        if qa_plots:
            render_quasi_static_qa(tvir, taps,
                                   out / f"{octt_name}.qa.pdf")
        snapshots.append({
            "index": idx,
            "source_file": str(files[spec.file_index]),
            "window_start_s": taps.window_start_s,
            "window_duration_s": taps.window_duration_s,
            "octt_file": octt_name,
            "v0_m_s": taps.v0_m_s,
            "fc_meas_hz": taps.fc_meas_hz,
            "doppler_resolution_hz": taps.doppler_resolution_hz,
            "captured_energy_fraction": taps.captured_energy_fraction,
            "normalization_db": taps.normalization_db,
            "dynamic_range_db": taps.dynamic_range_db,
            "taps": [
                {"delay_s": float(taps.delays_s[k]),
                 "amplitude": float(taps.amplitudes[k]),
                 "doppler_hz": float(taps.doppler_hz[k])}
                for k in range(taps.tap_count)
            ],
        })

    if len(snapshots) < n_snapshots:
        sys.stderr.write(
            f"[ensemble] only {len(snapshots)}/{n_snapshots} usable "
            "windows — no spare files left to substitute.\n")
    if not snapshots:
        raise ValueError("no usable windows: every candidate failed the "
                         "dynamic-range check")

    manifest = {
        "dataset": label,
        "dataset_dir": str(dataset_dir),
        "n_snapshots": len(snapshots),
        "record_duration_s": record_duration_s,
        "record_dt_s": record_dt_s,
        "extraction": {
            "max_taps": max_taps,
            "prominence_db": prominence_db,
            "window_duration_s": window_duration_s,
            "min_dynamic_range_db": min_dynamic_range_db,
        },
        "mean_v0_m_s": (sum(s["v0_m_s"] for s in snapshots)
                        / len(snapshots)),
        "snapshots": snapshots,
        "rejected": rejected,
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest
