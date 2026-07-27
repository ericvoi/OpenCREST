"""Build a quasi-static snapshot ensemble from a Watermark dataset
directory (one ``.octt`` per snapshot + ``manifest.json``).

Example:

    experiments/.venv/bin/python -m experiments.build_snapshot_ensemble \
        datasets/WatermarkV1/Watermark/input/channels/BCH1/mat \
        experiments/results/ensembles/BCH1 --n-snapshots 10 --plot
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from experiments.lib.snapshot_ensemble import build_ensemble
from experiments.lib.tap_trajectory import MAX_TAPS


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Extract N quasi-static snapshots from a Watermark "
                    "dataset into .octt files + manifest.json.")
    ap.add_argument("dataset_dir", type=Path,
                    help="directory holding the dataset's .mat files")
    ap.add_argument("out_dir", type=Path,
                    help="output directory for .octt files + manifest")
    ap.add_argument("--n-snapshots", type=int, default=10)
    ap.add_argument("--window-duration-s", type=float, default=None,
                    help="cap each snapshot's window length (default: "
                         "fill the allocated slot)")
    ap.add_argument("--max-taps", type=int, default=MAX_TAPS)
    ap.add_argument("--prominence-db", type=float, default=-30.0)
    ap.add_argument("--record-duration-s", type=float, default=30.0)
    ap.add_argument("--record-dt-s", type=float, default=1.0)
    ap.add_argument("--min-dynamic-range-db", type=float, default=20.0,
                    help="reject windows whose scattering function has "
                         "less peak-over-median dynamic range than this "
                         "(structureless data, default 20)")
    ap.add_argument("--pattern", default="*.mat",
                    help="glob for the sounding files to draw windows from. "
                         "Narrow to one hydrophone (e.g. 'BCH1_001.mat') so "
                         "the ensemble matches a single-geometry twin.")
    ap.add_argument("--plot", action="store_true",
                    help="write a QA figure per snapshot")
    args = ap.parse_args(argv)

    try:
        manifest = build_ensemble(
            args.dataset_dir, args.out_dir,
            n_snapshots=args.n_snapshots,
            window_duration_s=args.window_duration_s,
            max_taps=args.max_taps,
            prominence_db=args.prominence_db,
            record_duration_s=args.record_duration_s,
            record_dt_s=args.record_dt_s,
            min_dynamic_range_db=args.min_dynamic_range_db,
            pattern=args.pattern,
            qa_plots=args.plot,
        )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"{manifest['dataset']}: {manifest['n_snapshots']} snapshots "
          f"-> {args.out_dir}")
    print(f"  mean v0 {manifest['mean_v0_m_s']:+.3f} m/s "
          "(use for the geometric twin's radial velocity)")
    for s in manifest["snapshots"]:
        src = Path(s["source_file"]).name
        print(f"  snap{s['index']:02d}: {src} "
              f"[{s['window_start_s']:.1f}, "
              f"{s['window_start_s'] + s['window_duration_s']:.1f}] s  "
              f"{len(s['taps'])} taps, "
              f"captured {s['captured_energy_fraction']:.2f}, "
              f"v0 {s['v0_m_s']:+.2f} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
