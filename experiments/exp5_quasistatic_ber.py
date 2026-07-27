"""Experiment 5 — quasi-static BER vs SNR over a snapshot ensemble.

Sweeps raw (pre-ECC) and coded bit-error rate against channel gain_db
across the quasi-static snapshots of one Watermark dataset. Noise stays
fixed (Wenz sea state) while gain_db scales the signal, so gain_db is
the SNR axis up to a per-setup calibration offset. Each cell runs one
(snapshot, gain, seed) tuple: modem A fires PRBS evaluation messages
through the frozen replay channel; modem B scores the cargo against the
shared PRBS and prints error counts, which pool binomially.

Variance model: repeat seeds only average simulator-noise realisations
(BER through one frozen channel is nearly deterministic), so the error
bars come from the spread ACROSS snapshots — the ensemble carries the
channel-to-channel variance.

Detection censoring: BER is only reported for messages whose preamble
the modem synchronised on, so each gain point also records the
detection rate (reports / TX). Points with low detection rate are
biased optimistically — the figure carries the detection rate on a
secondary axis so the censored region is visible.

Zero-error cells plot at the one-sided floor 0.5/bits (the usual
"< 1 error" convention); the CSVs keep the true zero counts.

Inputs: an ensemble manifest built by ``build_snapshot_ensemble``.

Outputs (under ``--out``, default ``experiments/results/exp5/``):

  <cell>/scenario.yaml
  <cell>/<scenario>_summary.json
  <cell>/modem_a_cdc.log, modem_b_cdc.log
  <cell>/tx_log.csv              host-driver TX requests
  <cell>/cell_meta.json          {snapshot, gain_db, seed, octt_file, ...}

  ber_cells.csv                  long form: one row per cell
  ber_by_snapshot.csv            pooled across seeds per (gain, snapshot)
  ber_vs_gain.csv                per gain: mean/std across snapshots +
                                 pooled detection rate
  fig_ber_vs_gain.pdf            BER vs gain, log-y, error bars across
                                 snapshots; detection rate secondary axis

Modems must be pre-configured out of band (protocol parameters, forced/
known preamble content for EVAL scoring); the driver only sets the EVAL
cargo length and triggers transmissions.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Sequence

from experiments.lib import plotting
from experiments.lib.cdc_console import CdcConsole
from experiments.lib.runner import CellHandle, Sweep
from experiments.lib.watermark_twins import WATERMARK_TWINS, twin_params


REPO       = Path(__file__).resolve().parents[1]
CONFIG_DIR = REPO / "experiments" / "configs" / "exp5"
TEMPLATE   = CONFIG_DIR / "exp5_quasistatic_ber.yaml.j2"
TWIN_TEMPLATE = REPO / "experiments" / "configs" / "twin" / "geometric_twin.yaml.j2"

DEFAULT_GAINS_DB        = (-62.0, -58.0, -54.0, -50.0, -46.0)
DEFAULT_SEEDS           = 2
DEFAULT_PACKETS_PER_CELL = 15
DEFAULT_EVAL_LEN_BYTES  = 100
DEFAULT_SEA_STATE       = 3

DEFAULT_CADENCE_S             = 12.0   # ~2 s over the firmware minimum so
                                       # the receiver's TX/RX turnaround
                                       # clears before the next message
DEFAULT_FIRST_REQUEST_DELAY_S = 5.0
DEFAULT_RX_GRACE_S            = 8.0
DEFAULT_MAX_CELL_RUNTIME_S    = 300.0
SIGTERM_GRACE_S = 6.0

READY_PATTERN   = re.compile(r"Simulation running")
# Firmware prints (comm_print.c): "Uncoded BER: %hu/%hu, %.3f%%" then
# "Coded BER: %hu/%hu, %.3f%%" per evaluation message. Counts, not the
# percentages, are parsed so bits pool binomially.
EVAL_UNCODED_RE = re.compile(r"Uncoded BER:\s*(\d+)/(\d+)")
EVAL_CODED_RE   = re.compile(r"Coded BER:\s*(\d+)/(\d+)")
# Firmware prints (comm_print.c) "SNR: %.2f" once per received message,
# just before its evaluation report. A linear energy ratio, not dB.
SNR_RE          = re.compile(r"SNR:\s*([-+]?\d+(?:\.\d+)?)")


# ---------------------------------------------------------------------------
# Pure analysis primitives
# ---------------------------------------------------------------------------

@dataclass
class EvalReport:
    """One received evaluation message's error counts."""
    uncoded_errors: int
    uncoded_bits:   int
    coded_errors:   int
    coded_bits:     int


def parse_snr_values(lines: Iterable[str]) -> list[float]:
    """Every modem-reported ``SNR: <ratio>`` value (linear), in order."""
    out: list[float] = []
    for line in lines:
        if not isinstance(line, str):
            continue
        m = SNR_RE.search(line)
        if m:
            out.append(float(m.group(1)))
    return out


def parse_eval_reports(lines: Iterable[str]) -> list[EvalReport]:
    """Pair each ``Uncoded BER: e/n`` line with the next ``Coded BER``
    line. Unpaired leftovers (log cut mid-report) are dropped."""
    reports: list[EvalReport] = []
    pending: tuple[int, int] | None = None
    for line in lines:
        if not isinstance(line, str):
            continue
        mu = EVAL_UNCODED_RE.search(line)
        if mu:
            pending = (int(mu.group(1)), int(mu.group(2)))
            continue
        mc = EVAL_CODED_RE.search(line)
        if mc and pending is not None:
            reports.append(EvalReport(pending[0], pending[1],
                                      int(mc.group(1)), int(mc.group(2))))
            pending = None
    return reports


class _BerCounts:
    """Shared BER/detection accessors over pooled counts."""

    @property
    def uncoded_ber(self) -> float:
        return (self.uncoded_errors / self.uncoded_bits
                if self.uncoded_bits > 0 else float("nan"))

    @property
    def coded_ber(self) -> float:
        return (self.coded_errors / self.coded_bits
                if self.coded_bits > 0 else float("nan"))

    @property
    def detection_rate(self) -> float:
        return self.n_reports / self.n_tx if self.n_tx > 0 else float("nan")

    @property
    def snr_mean_db(self) -> float:
        """Mean modem-reported SNR in dB (averaged in the linear domain)."""
        return _snr_db(self.snr_linear_sum, self.snr_count)


@dataclass
class CellBer(_BerCounts):
    """Per-cell pooled counts. One cell = one (snapshot, gain, seed)."""
    snapshot:  int
    gain_db:   float
    seed:      int
    n_tx:      int
    n_reports: int
    uncoded_errors: int
    uncoded_bits:   int
    coded_errors:   int
    coded_bits:     int
    snr_linear_sum: float = 0.0   # sum of the modem's per-message SNR ratios
    snr_count:      int = 0       # number of SNR values that went into the sum


@dataclass
class SnapshotPoint(_BerCounts):
    """Counts pooled across seeds for one (gain, snapshot)."""
    gain_db:   float
    snapshot:  int
    n_tx:      int
    n_reports: int
    uncoded_errors: int
    uncoded_bits:   int
    coded_errors:   int
    coded_bits:     int
    snr_linear_sum: float = 0.0
    snr_count:      int = 0


@dataclass
class GainPoint:
    """Across-snapshot statistics for one gain value."""
    gain_db:     float
    n_snapshots: int          # snapshots with at least one report
    uncoded_mean: float
    uncoded_std:  float       # sample std across snapshots (0 if n < 2)
    coded_mean:   float
    coded_std:    float
    n_tx:        int
    n_reports:   int
    detection_rate: float     # pooled over all cells at this gain
    snr_mean_db: float = float("nan")   # mean modem SNR across snapshots (dB)


def pool_by_snapshot(cells: Sequence[CellBer]) -> list[SnapshotPoint]:
    """Sum counts across seeds within each (gain, snapshot). Pooled
    counts are the correct estimator for unequal-n binomial data."""
    buckets: dict[tuple[float, int], list[float]] = {}
    for c in cells:
        key = (float(c.gain_db), int(c.snapshot))
        b = buckets.setdefault(key, [0, 0, 0, 0, 0, 0, 0.0, 0])
        b[0] += c.n_tx
        b[1] += c.n_reports
        b[2] += c.uncoded_errors
        b[3] += c.uncoded_bits
        b[4] += c.coded_errors
        b[5] += c.coded_bits
        b[6] += c.snr_linear_sum
        b[7] += c.snr_count
    return [SnapshotPoint(g, s, *counts)
            for (g, s), counts in sorted(buckets.items())]


def _mean_std(values: Sequence[float]) -> tuple[float, float]:
    if not values:
        return float("nan"), float("nan")
    mean = sum(values) / len(values)
    if len(values) < 2:
        return mean, 0.0
    var = sum((v - mean) ** 2 for v in values) / (len(values) - 1)
    return mean, math.sqrt(var)


def gain_stats(points: Sequence[SnapshotPoint]) -> list[GainPoint]:
    """Per gain: mean/std of per-snapshot BER (each snapshot weighted
    equally — the ensemble is the variance source) + pooled detection
    rate. Snapshots with zero reports are excluded from the BER stats
    but still count against the detection rate."""
    by_gain: dict[float, list[SnapshotPoint]] = {}
    for p in points:
        by_gain.setdefault(p.gain_db, []).append(p)
    out: list[GainPoint] = []
    for gain, pts in sorted(by_gain.items()):
        detected = [p for p in pts if p.n_reports > 0]
        u_mean, u_std = _mean_std([p.uncoded_ber for p in detected])
        c_mean, c_std = _mean_std([p.coded_ber for p in detected])
        n_tx = sum(p.n_tx for p in pts)
        n_reports = sum(p.n_reports for p in pts)
        # Mean SNR across snapshots: each snapshot's linear-mean SNR
        # weighted equally, then converted to dB (matches the BER mean).
        snr_lin_means = [p.snr_linear_sum / p.snr_count
                         for p in detected if p.snr_count > 0]
        snr_db = (10.0 * math.log10(sum(snr_lin_means) / len(snr_lin_means))
                  if snr_lin_means else float("nan"))
        out.append(GainPoint(
            gain_db=gain,
            n_snapshots=len(detected),
            uncoded_mean=u_mean, uncoded_std=u_std,
            coded_mean=c_mean, coded_std=c_std,
            n_tx=n_tx, n_reports=n_reports,
            detection_rate=(n_reports / n_tx if n_tx else float("nan")),
            snr_mean_db=snr_db,
        ))
    return out


# ---------------------------------------------------------------------------
# Loaders for per-cell artifacts
# ---------------------------------------------------------------------------

def load_cell_meta(cell_dir: Path) -> dict:
    path = cell_dir / "cell_meta.json"
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return {}


def count_tx_requests(cell_dir: Path) -> int:
    path = cell_dir / "tx_log.csv"
    if not path.is_file():
        return 0
    with path.open() as fp:
        return max(0, sum(1 for _ in fp) - 1)     # minus header


def analyse_cell(cell_dir: Path,
                 outlier_ber_threshold: float | None = None) -> CellBer | None:
    """Materialise one cell's BER counts, or ``None`` for a cell that
    aborted before writing host-side artifacts.

    ``outlier_ber_threshold`` (e.g. 0.05) drops any single evaluation
    message whose *uncoded* BER exceeds it before pooling. Those are
    demodulation desyncs — whole messages garbled at normal SNR (the
    convolutional decoder amplifies them, so coded > uncoded) — not the
    channel's bit-level error rate. Removing them and averaging the rest
    gives the fading/noise BER without the sync-failure contamination.
    ``None`` keeps every message.
    """
    meta = load_cell_meta(cell_dir)
    if "gain_db" not in meta or "snapshot" not in meta:
        return None
    n_tx = count_tx_requests(cell_dir)
    if n_tx == 0:
        return None

    cdc_path = cell_dir / "modem_b_cdc.log"
    lines = (cdc_path.read_text(errors="replace").splitlines()
             if cdc_path.is_file() else [])
    reports = parse_eval_reports(lines)
    snrs = parse_snr_values(lines)
    n_received = len(reports)   # detection counts every received message

    if outlier_ber_threshold is not None:
        kept = [r for r in reports
                if r.uncoded_bits == 0
                or r.uncoded_errors / r.uncoded_bits <= outlier_ber_threshold]
        n_dropped = len(reports) - len(kept)
        if n_dropped:
            sys.stderr.write(
                f"[exp5] {cell_dir.name}: dropped {n_dropped}/{len(reports)} "
                f"outlier message(s) with uncoded BER > "
                f"{outlier_ber_threshold:.0%}\n")
        reports = kept

    if not reports and (not cdc_path.is_file()
                        or cdc_path.stat().st_size < 100):
        sys.stderr.write(
            f"[exp5] WARNING: {cell_dir.name}: {n_tx} TX requests but "
            f"modem_b_cdc.log is missing/tiny — CDC console likely failed "
            "to attach; this cell reads as 0% detection regardless of "
            "actual decode success.\n")

    return CellBer(
        snapshot  = int(meta["snapshot"]),
        gain_db   = float(meta["gain_db"]),
        seed      = int(meta.get("seed", -1)),
        n_tx      = n_tx,
        n_reports = n_received,
        uncoded_errors = sum(r.uncoded_errors for r in reports),
        uncoded_bits   = sum(r.uncoded_bits for r in reports),
        coded_errors   = sum(r.coded_errors for r in reports),
        coded_bits     = sum(r.coded_bits for r in reports),
        snr_linear_sum = sum(snrs),
        snr_count      = len(snrs),
    )


def analyse(out_dir: Path,
            outlier_ber_threshold: float | None = None) -> list[CellBer]:
    out: list[CellBer] = []
    for child in sorted(out_dir.iterdir()):
        if not child.is_dir():
            continue
        cell = analyse_cell(child, outlier_ber_threshold=outlier_ber_threshold)
        if cell is not None:
            out.append(cell)
    return out


# ---------------------------------------------------------------------------
# CSV writers
# ---------------------------------------------------------------------------

def write_cells_csv(cells: Sequence[CellBer], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = sorted(cells, key=lambda c: (c.gain_db, c.snapshot, c.seed))
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["gain_db", "snapshot", "seed", "n_tx", "n_reports",
                    "uncoded_errors", "uncoded_bits", "uncoded_ber",
                    "coded_errors", "coded_bits", "coded_ber",
                    "snr_db", "snr_n"])
        for c in rows:
            w.writerow([f"{c.gain_db:.1f}", c.snapshot, c.seed,
                        c.n_tx, c.n_reports,
                        c.uncoded_errors, c.uncoded_bits,
                        _fmt(c.uncoded_ber),
                        c.coded_errors, c.coded_bits,
                        _fmt(c.coded_ber),
                        _fmt(c.snr_mean_db), c.snr_count])


def write_by_snapshot_csv(points: Sequence[SnapshotPoint],
                          path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["gain_db", "snapshot", "n_tx", "n_reports",
                    "detection_rate",
                    "uncoded_errors", "uncoded_bits", "uncoded_ber",
                    "coded_errors", "coded_bits", "coded_ber",
                    "snr_db", "snr_n"])
        for p in points:
            w.writerow([f"{p.gain_db:.1f}", p.snapshot,
                        p.n_tx, p.n_reports, _fmt(p.detection_rate),
                        p.uncoded_errors, p.uncoded_bits,
                        _fmt(p.uncoded_ber),
                        p.coded_errors, p.coded_bits,
                        _fmt(p.coded_ber),
                        _fmt(p.snr_mean_db), p.snr_count])


def write_gain_csv(stats: Sequence[GainPoint], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["gain_db", "n_snapshots",
                    "uncoded_ber_mean", "uncoded_ber_std",
                    "coded_ber_mean", "coded_ber_std",
                    "n_tx", "n_reports", "detection_rate",
                    "snr_db_mean"])
        for g in stats:
            w.writerow([f"{g.gain_db:.1f}", g.n_snapshots,
                        _fmt(g.uncoded_mean), _fmt(g.uncoded_std),
                        _fmt(g.coded_mean), _fmt(g.coded_std),
                        g.n_tx, g.n_reports, _fmt(g.detection_rate),
                        _fmt(g.snr_mean_db)])


def _fmt(v: float) -> str:
    return "" if math.isnan(v) else f"{v:.6g}"


def _snr_db(linear_sum: float, count: int) -> float:
    """Mean SNR in dB from a running sum of the modem's linear SNR
    ratios. Averaging is done in the linear (power) domain, then
    converted, so it is a true mean SNR rather than a mean of dB."""
    if count <= 0 or linear_sum <= 0.0:
        return float("nan")
    return 10.0 * math.log10(linear_sum / count)


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

def render_ber_figure(stats: Sequence[GainPoint], savepath: Path,
                      *, floor_bits_hint: int = 12_000) -> None:
    """BER vs gain, log-y, error bars = std across snapshots. Zero-mean
    points draw at the 0.5/bits one-sided floor with open markers."""
    import matplotlib.pyplot as plt          # local: plotting is optional
    plotting.apply_paper_style()
    fig, ax = plt.subplots(figsize=(7.0, 4.2))

    pts = [g for g in stats if g.n_snapshots > 0]
    if pts:
        xs = [g.gain_db for g in pts]
        floor = 0.5 / max(floor_bits_hint, 1)

        def clipped(values):
            return [max(v, floor) for v in values]

        for label, mean_key, std_key, style in (
                ("raw (pre-ECC)", "uncoded_mean", "uncoded_std", "-o"),
                ("coded (post-ECC)", "coded_mean", "coded_std", "--s")):
            ys = clipped([getattr(g, mean_key) for g in pts])
            yerr = [getattr(g, std_key) for g in pts]
            # Asymmetric clip so error bars never cross zero on a log axis.
            lower = [max(y - e, floor * 0.9) for y, e in zip(ys, yerr)]
            ax.errorbar(xs, ys,
                        yerr=[[y - lo for y, lo in zip(ys, lower)], yerr],
                        fmt=style, capsize=3, markersize=4, label=label)

        ax.set_yscale("log")
        ax_det = ax.twinx()
        ax_det.plot(xs, [g.detection_rate for g in pts],
                    ":", color="gray", alpha=0.7, label="detection rate")
        ax_det.set_ylabel("Detection rate", color="gray")
        ax_det.set_ylim(-0.02, 1.05)
        ax_det.tick_params(axis="y", colors="gray")
        ax.legend(loc="best", frameon=False)

    ax.set_xlabel("Channel gain (dB)")
    ax.set_ylabel("Bit error rate")
    ax.grid(True, alpha=0.3)
    savepath.parent.mkdir(parents=True, exist_ok=True)
    plotting.save_figure(fig, savepath)


# ---------------------------------------------------------------------------
# CDC driver — per-cell pre/post hooks
# ---------------------------------------------------------------------------

class _EvalDriver:
    """Owns the CDC consoles and the EVAL TX-loop thread for one cell."""

    def __init__(self,
                 modem_a_serial: str,
                 modem_b_serial: str,
                 *,
                 snapshot: int,
                 gain_db: float,
                 seed: int,
                 octt_file: str,
                 eval_len_bytes: int,
                 cadence_s: float,
                 first_request_delay_s: float,
                 packets_per_cell: int,
                 rx_grace_s: float,
                 cdc_factory: Callable[..., CdcConsole] = CdcConsole.attach,
                 ready_timeout_s: float = 15.0) -> None:
        self.modem_a_serial = modem_a_serial
        self.modem_b_serial = modem_b_serial
        self.snapshot  = int(snapshot)
        self.gain_db   = float(gain_db)
        self.seed      = int(seed)
        self.octt_file = octt_file
        self.eval_len_bytes = int(eval_len_bytes)
        self.cadence_s = float(cadence_s)
        self.first_request_delay_s = float(first_request_delay_s)
        self.packets_per_cell = int(packets_per_cell)
        self.rx_grace_s = float(rx_grace_s)
        self._cdc_factory = cdc_factory
        self._ready_timeout = ready_timeout_s

        self._consoles: dict[str, list[CdcConsole]] = {}
        self._tx_counts: dict[str, list[tuple[int, int]]] = {}
        self._done_evts: dict[str, threading.Event] = {}
        self._stop_evts: dict[str, threading.Event] = {}
        self._threads: dict[str, threading.Thread] = {}

    # ---- Sweep hooks ---------------------------------------------------

    def pre_run(self, handle: CellHandle) -> None:
        self._tx_counts[handle.cell_id] = []
        self._wait_for_ready(handle.cell_dir / "stdout.log")
        cdc_a = self._cdc_factory(
            modem_id="modem_a", usb_serial=self.modem_a_serial,
            log_path=handle.cell_dir / "modem_a_cdc.log")
        cdc_b = self._cdc_factory(
            modem_id="modem_b", usb_serial=self.modem_b_serial,
            log_path=handle.cell_dir / "modem_b_cdc.log")
        self._consoles[handle.cell_id] = [cdc_a, cdc_b]

        try:
            for cdc in (cdc_a, cdc_b):
                if not cdc.verify_main_menu(timeout_s=2.0):
                    raise RuntimeError(
                        f"{cdc.modem_id}: CDC attached but no 'Main Menu' "
                        "echo within 2.0 s — modem unresponsive or wedged.")
            cdc_a.set_eval_message_len(self.eval_len_bytes)
        except BaseException:
            for cdc in (cdc_a, cdc_b):
                try:
                    cdc.detach()
                except Exception:
                    pass
            self._consoles.pop(handle.cell_id, None)
            raise

        done = threading.Event(); stop = threading.Event()
        self._done_evts[handle.cell_id] = done
        self._stop_evts[handle.cell_id] = stop
        worker = threading.Thread(
            target=self._tx_loop,
            args=(handle.cell_id, cdc_a, cdc_b, done, stop),
            name=f"exp5-tx-{handle.cell_id[:8]}",
            daemon=True)
        self._threads[handle.cell_id] = worker
        worker.start()

    def post_run(self, handle: CellHandle) -> None:
        stop = self._stop_evts.pop(handle.cell_id, None)
        if stop is not None:
            stop.set()
        thread = self._threads.pop(handle.cell_id, None)
        if thread is not None:
            thread.join(timeout=5.0)

        tx_log = self._tx_counts.pop(handle.cell_id, [])
        self._write_tx_log(handle.cell_dir, tx_log)
        self._write_cell_meta(handle.cell_dir)

        if self.rx_grace_s > 0:
            time.sleep(self.rx_grace_s)

        for console in self._consoles.pop(handle.cell_id, []):
            try:
                console.detach()
            except Exception:
                pass
        self._done_evts.pop(handle.cell_id, None)

    def cell_done(self, handle: CellHandle) -> bool:
        evt = self._done_evts.get(handle.cell_id)
        return bool(evt and evt.is_set())

    # ---- internals -----------------------------------------------------

    def _wait_for_ready(self, stdout: Path) -> None:
        deadline = time.monotonic() + self._ready_timeout
        while time.monotonic() < deadline:
            if stdout.is_file():
                try:
                    text = stdout.read_text(errors="replace")
                except OSError:
                    text = ""
                if READY_PATTERN.search(text):
                    return
            time.sleep(0.1)
        sys.stderr.write(
            "[exp5] simulator did not print 'Simulation running' within "
            f"{self._ready_timeout:.1f}s — proceeding anyway; first few "
            "TXs may be lost to startup.\n")

    def _tx_loop(self, cell_id: str, cdc_a: CdcConsole, cdc_b: CdcConsole,
                 done: threading.Event, stop: threading.Event) -> None:
        tx_log = self._tx_counts[cell_id]
        if self.first_request_delay_s > 0.0:
            stop.wait(self.first_request_delay_s)
        seq = 0
        while seq < self.packets_per_cell and not stop.is_set():
            if not cdc_a.is_reader_alive() or not cdc_b.is_reader_alive():
                which = "A" if not cdc_a.is_reader_alive() else "B"
                sys.stderr.write(f"[exp5] cell={cell_id[:8]} seq={seq}: "
                                 f"modem-{which} CDC reader died — aborting "
                                 "cell to avoid corrupted BER counts.\n")
                break
            now_ns = time.monotonic_ns()
            try:
                cdc_a.send_eval_tx()
            except Exception as exc:                        # noqa: BLE001
                sys.stderr.write(f"[exp5] cell={cell_id[:8]} seq={seq} "
                                 f"TX failed: {exc}\n")
            tx_log.append((seq, now_ns))
            seq += 1
            stop.wait(self.cadence_s)
        done.set()

    def _write_tx_log(self, cell_dir: Path,
                      log: list[tuple[int, int]]) -> None:
        with (cell_dir / "tx_log.csv").open("w", newline="") as fp:
            w = csv.writer(fp)
            w.writerow(["request_id", "request_ts_ns"])
            for seq, ts in log:
                w.writerow([seq, ts])

    def _write_cell_meta(self, cell_dir: Path) -> None:
        meta = dict(
            snapshot         = self.snapshot,
            gain_db          = self.gain_db,
            seed             = self.seed,
            octt_file        = self.octt_file,
            eval_len_bytes   = self.eval_len_bytes,
            cadence_s        = self.cadence_s,
            packets_per_cell = self.packets_per_cell,
        )
        (cell_dir / "cell_meta.json").write_text(json.dumps(meta, indent=2))


# ---------------------------------------------------------------------------
# Sweep wiring
# ---------------------------------------------------------------------------

def _gain_tag(gain_db: float) -> str:
    """Filesystem-safe gain label: -52.5 -> m52p5, 3.0 -> p3p0."""
    sign = "m" if gain_db < 0 else "p"
    return f"{sign}{abs(gain_db):.1f}".replace(".", "p")


def _cell_is_valid(cell_dir: Path) -> bool:
    """Usable-cell heuristic: at least one TX logged and a non-trivial
    modem-B CDC log (catches silent CDC-attach failures that would read
    as 0% detection). A cell that ran but scored nothing is still a valid
    measurement — its 0 reports are a real detection rate, not a retry
    trigger."""
    if count_tx_requests(cell_dir) < 1:
        return False
    cdc_log = cell_dir / "modem_b_cdc.log"
    try:
        return cdc_log.is_file() and cdc_log.stat().st_size >= 100
    except OSError:
        return False


def _per_cell_sweep(*,
                    snapshot: int,
                    gain_db: float,
                    seed: int,
                    trajectory_file: Path,
                    record_dt_s: float,
                    sea_state: int,
                    out_root: Path,
                    binary: Path | str,
                    modem_a_serial: str,
                    modem_b_serial: str,
                    eval_len_bytes: int,
                    cadence_s: float,
                    first_request_delay_s: float,
                    max_cell_runtime_s: float,
                    rx_grace_s: float,
                    retries: int,
                    packets_per_cell: int) -> None:
    driver = _EvalDriver(
        modem_a_serial=modem_a_serial,
        modem_b_serial=modem_b_serial,
        snapshot=snapshot,
        gain_db=gain_db,
        seed=seed,
        octt_file=str(trajectory_file),
        eval_len_bytes=eval_len_bytes,
        cadence_s=cadence_s,
        first_request_delay_s=first_request_delay_s,
        packets_per_cell=packets_per_cell,
        rx_grace_s=rx_grace_s,
    )
    sweep = Sweep(
        template_path=TEMPLATE,
        parameters={
            "snapshot": [snapshot],
            "gain_db":  [gain_db],
            "seed":     [seed],
        },
        extra_params=dict(
            gain_tag        = _gain_tag(gain_db),
            trajectory_file = str(trajectory_file),
            record_dt_s     = record_dt_s,
            sea_state       = sea_state,
            modem_a_serial  = modem_a_serial,
            modem_b_serial  = modem_b_serial,
        ),
        binary=binary,
        out_dir=out_root,
        duration_s=float("inf"),          # cell_done drives shutdown
        parallel=1,
        sigterm_grace_s=SIGTERM_GRACE_S,
        retries=retries,
        max_cell_runtime_s=max_cell_runtime_s,
        pre_run=driver.pre_run,
        post_run=driver.post_run,
        stop_condition=driver.cell_done,
    )
    sweep.run()


def _twin_cell_sweep(*,
                     dataset: str,
                     realization: int,
                     gain_db: float,
                     seed: int,
                     velocity_override: float | None,
                     sea_state: int,
                     max_bounces: int,
                     out_root: Path,
                     binary: Path | str,
                     modem_a_serial: str,
                     modem_b_serial: str,
                     eval_len_bytes: int,
                     cadence_s: float,
                     first_request_delay_s: float,
                     max_cell_runtime_s: float,
                     rx_grace_s: float,
                     retries: int,
                     packets_per_cell: int) -> None:
    """One geometric-twin cell: same EVAL/CDC driver as replay, but the
    channel is the dataset's geometric twin at swept ``gain_db``. Each
    ``realization`` is an independent noise seed; its index is stored as
    the ``snapshot`` field so the across-snapshot statistics machinery
    produces error bars over the realizations."""
    driver = _EvalDriver(
        modem_a_serial=modem_a_serial,
        modem_b_serial=modem_b_serial,
        snapshot=realization,
        gain_db=gain_db,
        seed=seed,
        octt_file=f"twin_{dataset}",
        eval_len_bytes=eval_len_bytes,
        cadence_s=cadence_s,
        first_request_delay_s=first_request_delay_s,
        packets_per_cell=packets_per_cell,
        rx_grace_s=rx_grace_s,
    )
    tp = twin_params(
        dataset,
        seed=seed,
        channel_gain_db=gain_db,
        radial_velocity_override_m_s=velocity_override,
        modem_a_serial=modem_a_serial,
        modem_b_serial=modem_b_serial,
    )
    sweep = Sweep(
        template_path=TWIN_TEMPLATE,
        parameters={
            "realization": [realization],
            "gain_db":     [gain_db],
            "seed":        [seed],
        },
        extra_params=dict(tp, sea_state=sea_state, gain_tag=_gain_tag(gain_db),
                          max_bounces=max_bounces),
        binary=binary,
        out_dir=out_root,
        duration_s=float("inf"),
        parallel=1,
        sigterm_grace_s=SIGTERM_GRACE_S,
        retries=retries,
        max_cell_runtime_s=max_cell_runtime_s,
        pre_run=driver.pre_run,
        post_run=driver.post_run,
        stop_condition=driver.cell_done,
    )
    sweep.run()


RETRY_BACKOFF_S = 3.0


HARNESS_MAX_ATTEMPTS = 5


def _cell_report_count(cell_dir: Path) -> int:
    """Number of parsed EVAL reports in a cell's modem-B CDC log."""
    cdc = cell_dir / "modem_b_cdc.log"
    if not cdc.is_file():
        return 0
    return len(parse_eval_reports(
        cdc.read_text(errors="replace").splitlines()))


def _quarantine(cell: Path, attempt: int) -> None:
    q = cell.with_name(f"{cell.name}_failed_attempt{attempt}")
    try:
        cell.rename(q)
        sys.stderr.write(f"[exp5]   quarantined {cell.name} -> {q.name}\n")
    except OSError as e:
        sys.stderr.write(f"[exp5]   could not quarantine {cell.name}: {e}\n")


def _per_cell_sweep_with_retry(*, sweep_fn: Callable[..., None] = _per_cell_sweep,
                               detect_attempts: int = 1, **kwargs) -> None:
    """Run one cell with two independent, bounded retry policies:

    * **Harness failure** (crash, silent CDC-attach failure, wedged modem,
      watchdog kill — i.e. no captured CDC log): retried up to
      ``HARNESS_MAX_ATTEMPTS`` times, each attempt quarantined; if none
      succeed the cell is abandoned with no data point.
    * **Zero detections on a clean run**: the modem is stochastic
      run-to-run (real ADC/timing/AGC), so a cell that locks on nothing
      can lock on a re-run of the *same* channel+seed. Re-run up to
      ``detect_attempts`` times; keep the first cell that scores >=1
      report. If all ``detect_attempts`` score nothing, keep the last one
      as a genuine detection failure (a real, kept data point).

    ``detect_attempts=1`` (default) keeps the first result either way — no
    detection retry. Values of 3-5 reduce snapshot censoring at harsh
    gains, at the cost of biasing the *reported detection rate* high
    (early-stop on first success); BER is the intended deliverable, not
    detection rate. Ctrl+C aborts."""
    out_root: Path = kwargs["out_root"]
    cell_tag = kwargs.get("snapshot", kwargs.get("realization", "?"))
    label = (f"cell={cell_tag} g={kwargs['gain_db']:.1f}dB "
             f"seed={kwargs['seed']}")

    attempt = 0
    detect_misses = 0
    harness_fails = 0
    while True:
        attempt += 1
        cells_before = {p.name for p in out_root.iterdir() if p.is_dir()}

        sweep_exc: BaseException | None = None
        try:
            sweep_fn(**kwargs)
        except KeyboardInterrupt:
            raise
        except BaseException as exc:                        # noqa: BLE001
            sweep_exc = exc
            sys.stderr.write(f"[exp5] {label} attempt={attempt}: sweep "
                             f"raised {type(exc).__name__}: {exc}\n")

        new_cells = sorted(
            out_root / name for name in
            ({p.name for p in out_root.iterdir() if p.is_dir()}
             - cells_before))
        good = [c for c in new_cells if _cell_is_valid(c)]

        # --- Harness failure: crash or no captured CDC log ---------------
        if sweep_exc is not None or not good:
            harness_fails += 1
            for c in new_cells:
                _quarantine(c, attempt)
            if harness_fails >= HARNESS_MAX_ATTEMPTS:
                sys.stderr.write(
                    f"[exp5] {label}: {harness_fails} harness failures — "
                    "giving up, no data point.\n")
                return
            sys.stderr.write(
                f"[exp5] {label} attempt={attempt}: harness failure "
                "(crash / CDC / wedge) — retrying.\n")
            time.sleep(RETRY_BACKOFF_S)
            continue

        # --- Clean run with a CDC log -----------------------------------
        cell = good[0]
        reports = _cell_report_count(cell)
        if reports >= 1:
            if attempt > 1:
                sys.stderr.write(f"[exp5] {label}: detected on attempt "
                                 f"{attempt} ({reports} reports).\n")
            return                                  # keep the detected cell

        # Zero detections on a clean run.
        detect_misses += 1
        if detect_misses >= detect_attempts:
            if detect_attempts > 1:
                sys.stderr.write(
                    f"[exp5] {label}: 0 detections in {detect_misses} "
                    "attempt(s) — keeping as a genuine detection failure.\n")
            return                                  # keep the miss as a point
        _quarantine(cell, attempt)
        sys.stderr.write(
            f"[exp5] {label} attempt={attempt}: 0 detections — retrying "
            f"for a lock (up to {detect_attempts}).\n")
        time.sleep(RETRY_BACKOFF_S)


def _print_summary(stats: Sequence[GainPoint]) -> None:
    sys.stderr.write("[exp5] BER vs gain (mean ± std across snapshots):\n")
    for g in stats:
        if g.n_snapshots == 0:
            sys.stderr.write(f"  g={g.gain_db:7.1f} dB   no reports "
                             f"(detection {g.detection_rate:.2f}, "
                             f"n_tx={g.n_tx})\n")
            continue
        snr = ("  --  " if math.isnan(g.snr_mean_db)
               else f"{g.snr_mean_db:5.1f}dB")
        sys.stderr.write(
            f"  g={g.gain_db:7.1f} dB   SNR={snr}   "
            f"raw={g.uncoded_mean:.3e}±{g.uncoded_std:.1e}   "
            f"coded={g.coded_mean:.3e}±{g.coded_std:.1e}   "
            f"det={g.detection_rate:.2f}   "
            f"snapshots={g.n_snapshots}\n")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_gains(tokens: Sequence[str]) -> list[float]:
    """Flatten space- and/or comma-separated gain tokens to floats.

    Gains are collected with ``nargs='+'`` because argparse rejects a
    bare ``-62,-58`` token — it starts with ``-`` but doesn't match the
    negative-number pattern, so argparse reads it as an option flag. The
    space form ``--gains -62 -58`` works (each token is a negative
    number); a single ``--gains=-62,-58`` token is comma-split here.
    """
    out: list[float] = []
    for tok in tokens:
        out.extend(float(g) for g in str(tok).split(",") if g.strip())
    return out


def _parse_snapshots(text: str | None, manifest: dict) -> list[dict]:
    snaps = manifest["snapshots"]
    if text is None:
        return snaps
    wanted = {int(s) for s in text.split(",") if s.strip()}
    picked = [s for s in snaps if int(s["index"]) in wanted]
    missing = wanted - {int(s["index"]) for s in picked}
    if missing:
        raise ValueError(f"snapshot indices {sorted(missing)} not in "
                         "the manifest")
    return picked


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--manifest", type=Path,
                   help="ensemble manifest.json from "
                        "build_snapshot_ensemble (required unless "
                        "--analysis-only)")
    p.add_argument("--out", default="experiments/results/exp5",
                   help="output directory for sweep artifacts")
    p.add_argument("--binary", default=str(REPO / "build" / "openCREST"),
                   help="path to the openCREST simulator binary")
    p.add_argument("--gains", nargs="+", metavar="GAIN_DB",
                   default=[f"{g:g}" for g in DEFAULT_GAINS_DB],
                   help="channel gain_db values, space-separated: "
                        "--gains -62 -58 -54 (the leading minus needs the "
                        "space form; --gains=-62,-58 also works). Default "
                        f"{' '.join(f'{g:g}' for g in DEFAULT_GAINS_DB)}. "
                        "Calibrate the range so the raw-BER waterfall is "
                        "inside it.")
    p.add_argument("--snapshots", default=None,
                   help="comma-separated snapshot indices (default: all "
                        "in the manifest)")
    p.add_argument("--seeds", type=int, default=DEFAULT_SEEDS,
                   help=f"repeat seeds per (snapshot, gain) (default "
                        f"{DEFAULT_SEEDS}; repeats only average noise "
                        "realisations — snapshots carry the variance)")
    p.add_argument("--packets-per-cell", type=int,
                   default=DEFAULT_PACKETS_PER_CELL,
                   help=f"EVAL messages per cell (default "
                        f"{DEFAULT_PACKETS_PER_CELL})")
    p.add_argument("--eval-len-bytes", type=int,
                   default=DEFAULT_EVAL_LEN_BYTES,
                   help="PRBS cargo length in bytes, firmware range 1..480 "
                        f"(default {DEFAULT_EVAL_LEN_BYTES})")
    p.add_argument("--sea-state", type=int, default=DEFAULT_SEA_STATE,
                   help=f"fixed Wenz sea state (default {DEFAULT_SEA_STATE})")
    p.add_argument("--cadence-s", type=float, default=DEFAULT_CADENCE_S)
    p.add_argument("--first-request-delay-s", type=float,
                   default=DEFAULT_FIRST_REQUEST_DELAY_S)
    p.add_argument("--rx-grace-s", type=float, default=DEFAULT_RX_GRACE_S)
    p.add_argument("--max-cell-runtime-s", type=float,
                   default=DEFAULT_MAX_CELL_RUNTIME_S)
    p.add_argument("--modem-a-serial", default="OA-2-1")
    p.add_argument("--modem-b-serial", default="OA-2-2")
    p.add_argument("--retries", type=int, default=0)
    p.add_argument("--analysis-only", action="store_true",
                   help="don't run the sweep — only post-process "
                        "artifacts already present in --out")
    p.add_argument("--geometric-twin", metavar="DATASET", default=None,
                   help="run the geometric TWIN of DATASET (e.g. BCH1) "
                        "instead of replay — same EVAL driver, gains, noise; "
                        "no --manifest needed. See lib/watermark_twins.py.")
    p.add_argument("--realizations", type=int, default=10,
                   help="twin only: independent noise realizations per gain "
                        "(error bars span these, default 10)")
    p.add_argument("--twin-max-bounces", type=int, default=2,
                   help="image-method reflection order for the geometric "
                        "twin (2=5 rays default, 4=9 rays)")
    p.add_argument("--twin-velocity-override", type=float, default=None,
                   metavar="M_S",
                   help="twin only: radial velocity (m/s) — pass the "
                        "ensemble manifest's mean_v0_m_s so the twin moves "
                        "at the dataset's measured Doppler")
    p.add_argument("--outlier-ber-threshold", type=float, default=None,
                   help="drop any evaluation message whose uncoded BER "
                        "exceeds this (e.g. 0.05) before averaging — removes "
                        "whole-message demodulation desyncs that are not the "
                        "channel BER. Off by default.")
    p.add_argument("--detect-attempts", type=int, default=1,
                   help="re-run a cell that scored 0 detections up to this "
                        "many times hoping for a lock (the modem is "
                        "stochastic run-to-run). 1 = accept first result "
                        "(default). 3-5 reduces snapshot censoring at harsh "
                        "gains but biases the reported detection rate high.")
    args = p.parse_args(argv)

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.analysis_only:
        gains = _parse_gains(args.gains)

        if args.geometric_twin is not None:
            if args.geometric_twin not in WATERMARK_TWINS:
                sys.stderr.write(
                    f"[exp5] unknown twin dataset {args.geometric_twin!r}; "
                    f"known: {sorted(WATERMARK_TWINS)}\n")
                return 2
            for gain in gains:
                for realization in range(args.realizations):
                    _per_cell_sweep_with_retry(
                        sweep_fn=_twin_cell_sweep,
                        detect_attempts=args.detect_attempts,
                        dataset=args.geometric_twin,
                        realization=realization,
                        gain_db=gain,
                        seed=realization,
                        velocity_override=args.twin_velocity_override,
                        sea_state=args.sea_state,
                        max_bounces=args.twin_max_bounces,
                        out_root=out_dir,
                        binary=args.binary,
                        modem_a_serial=args.modem_a_serial,
                        modem_b_serial=args.modem_b_serial,
                        eval_len_bytes=args.eval_len_bytes,
                        cadence_s=args.cadence_s,
                        first_request_delay_s=args.first_request_delay_s,
                        max_cell_runtime_s=args.max_cell_runtime_s,
                        rx_grace_s=args.rx_grace_s,
                        retries=args.retries,
                        packets_per_cell=args.packets_per_cell,
                    )
        else:
            if args.manifest is None:
                p.error("--manifest is required unless --analysis-only "
                        "or --geometric-twin")
            manifest = json.loads(Path(args.manifest).read_text())
            manifest_dir = Path(args.manifest).resolve().parent
            try:
                snapshots = _parse_snapshots(args.snapshots, manifest)
            except ValueError as exc:
                sys.stderr.write(f"[exp5] {exc}\n")
                return 2
            record_dt_s = float(manifest.get("record_dt_s", 1.0))

            for gain in gains:
                for snap in snapshots:
                    for seed in range(args.seeds):
                        _per_cell_sweep_with_retry(
                            detect_attempts=args.detect_attempts,
                            snapshot=int(snap["index"]),
                            gain_db=gain,
                            seed=seed,
                            trajectory_file=manifest_dir / snap["octt_file"],
                            record_dt_s=record_dt_s,
                            sea_state=args.sea_state,
                            out_root=out_dir,
                            binary=args.binary,
                            modem_a_serial=args.modem_a_serial,
                            modem_b_serial=args.modem_b_serial,
                            eval_len_bytes=args.eval_len_bytes,
                            cadence_s=args.cadence_s,
                            first_request_delay_s=args.first_request_delay_s,
                            max_cell_runtime_s=args.max_cell_runtime_s,
                            rx_grace_s=args.rx_grace_s,
                            retries=args.retries,
                            packets_per_cell=args.packets_per_cell,
                        )

    cells = analyse(out_dir, outlier_ber_threshold=args.outlier_ber_threshold)
    if not cells:
        sys.stderr.write(f"[exp5] no analysable cells found in {out_dir} "
                         "— aborting\n")
        return 3

    points = pool_by_snapshot(cells)
    stats = gain_stats(points)

    write_cells_csv(cells, out_dir / "ber_cells.csv")
    write_by_snapshot_csv(points, out_dir / "ber_by_snapshot.csv")
    write_gain_csv(stats, out_dir / "ber_vs_gain.csv")
    floor_bits = max((p.uncoded_bits for p in points), default=12_000)
    render_ber_figure(stats, out_dir / "fig_ber_vs_gain.pdf",
                      floor_bits_hint=max(floor_bits, 1))
    _print_summary(stats)
    return 0


if __name__ == "__main__":
    sys.exit(main())
