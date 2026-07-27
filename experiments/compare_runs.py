"""Overlay BER vs modem-reported SNR for two or more exp5 sweeps — the
replay-vs-geometric-twin figure. Each run is analysed with exp5's own
pipeline, so the numbers match the per-run CSVs exactly.

    experiments/.venv/bin/python -m experiments.compare_runs \
        --out experiments/results/fig_replay_vs_twin.pdf \
        replay=experiments/results/exp5_bch1_replay \
        geometric=experiments/results/exp5_bch1_twin

Each positional arg is ``label=path``. The x axis is the modem-reported
SNR in dB (``snr_db_mean``): it is the same physical quantity for every
run, so replay and geometric-twin sweeps land on a common axis with no
per-arm gain calibration — the different ``gain_db`` ranges the two arms
need (the twin carries range-dependent spreading loss the peak-normalised
replay does not) are absorbed automatically. Pass ``--x gain`` to fall
back to the raw ``gain_db`` axis instead.

Only the uncoded (pre-ECC) BER is plotted: the coded curve sits on the
0.5/N floor across the whole sweep (post-ECC errors track detection, not
SNR), so it carries no channel information here. Error bars default to
the standard error of the ensemble mean (``std/sqrt(n_snapshots)``) — the
uncertainty in the plotted mean — rather than the raw across-snapshot
spread, which is dominated by real channel-to-channel heterogeneity.
Pass ``--err std`` for the full spread or ``--err none`` to drop bars.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from experiments.exp5_quasistatic_ber import (analyse, gain_stats,
                                              pool_by_snapshot)


def _parse_run(arg: str) -> tuple[str, Path]:
    if "=" not in arg:
        raise ValueError(f"run must be label=path, got {arg!r}")
    label, path = arg.split("=", 1)
    return label.strip(), Path(path).expanduser()


def _xy(stats, x_axis: str):
    """(x, points) for a run: x is modem SNR (dB) or gain, sorted by x,
    dropping points with no BER (or no SNR when x is snr)."""
    pts = [g for g in stats if g.n_snapshots > 0]
    if x_axis == "snr":
        pts = [g for g in pts if not math.isnan(g.snr_mean_db)]
        xs = [g.snr_mean_db for g in pts]
    else:
        xs = [g.gain_db for g in pts]
    order = sorted(range(len(pts)), key=lambda i: xs[i])
    return [xs[i] for i in order], [pts[i] for i in order]


def render_overlay(runs: list[tuple[str, list]], savepath: Path,
                   *, x_axis: str = "snr", err_mode: str = "sem",
                   floor_bits: int = 12_000) -> None:
    import matplotlib.pyplot as plt

    from experiments.lib import plotting
    plotting.apply_paper_style()
    fig, ax = plt.subplots()
    floor = 0.5 / max(floor_bits, 1)

    # One colour per run; uncoded BER only (the coded curve is floor-bound).
    cmap = plt.get_cmap("tab10")
    for i, (label, stats) in enumerate(runs):
        xs, pts = _xy(stats, x_axis)
        if not pts:
            sys.stderr.write(f"[compare] run {label!r} has no plottable "
                             f"points on the {x_axis} axis\n")
            continue
        colour = cmap(i % 10)

        def clip(vals):
            return [max(v, floor) for v in vals]

        raw = clip([g.uncoded_mean for g in pts])
        if err_mode == "none":
            raw_e = [0.0 for _ in pts]
        elif err_mode == "std":
            raw_e = [g.uncoded_std for g in pts]
        else:  # "sem": standard error of the across-snapshot mean
            raw_e = [g.uncoded_std / math.sqrt(g.n_snapshots)
                     if g.n_snapshots > 1 else 0.0 for g in pts]
        lo = [max(y - e, floor * 0.9) for y, e in zip(raw, raw_e)]
        ax.errorbar(xs, raw,
                    yerr=[[y - l for y, l in zip(raw, lo)], raw_e],
                    fmt="-o", color=colour, capsize=3, markersize=4,
                    label=label)

    ax.set_yscale("log")
    ax.set_xlabel("Modem-reported SNR (dB)" if x_axis == "snr"
                  else "Channel gain (dB)  →  increasing SNR")
    ax.set_ylabel("Bit error rate")
    ax.axhline(floor, color="gray", lw=0.6, ls=":", alpha=0.6)
    ax.legend(loc="best", frameon=False)
    fig.tight_layout()
    fig.savefig(savepath)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+", metavar="LABEL=PATH")
    ap.add_argument("--out", type=Path,
                    default=Path("experiments/results/fig_compare.pdf"))
    ap.add_argument("--x", choices=("snr", "gain"), default="snr",
                    help="x axis: modem-reported SNR (default) or gain_db")
    ap.add_argument("--err", choices=("sem", "std", "none"), default="sem",
                    help="error bars: standard error of the mean (default), "
                         "full across-snapshot std, or none")
    ap.add_argument("--outlier-ber-threshold", type=float, default=None,
                    help="drop evaluation messages with uncoded BER above "
                         "this (e.g. 0.05) before pooling — matches exp5's "
                         "own outlier removal. Off by default.")
    args = ap.parse_args(argv)

    runs: list[tuple[str, list]] = []
    for arg in args.runs:
        try:
            label, path = _parse_run(arg)
        except ValueError as exc:
            ap.error(str(exc))
        if not path.is_dir():
            sys.stderr.write(f"[compare] not a directory: {path}\n")
            return 2
        stats = gain_stats(pool_by_snapshot(
            analyse(path, outlier_ber_threshold=args.outlier_ber_threshold)))
        runs.append((label, stats))
        detected = sum(g.n_snapshots for g in stats)
        sys.stderr.write(
            f"[compare] {label}: {len(stats)} gains, "
            f"{detected} detected (snapshot,gain) points\n")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    render_overlay(runs, args.out, x_axis=args.x, err_mode=args.err)
    sys.stderr.write(f"[compare] wrote {args.out}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
