"""Experiment 3 — JANUS PER vs range, static-range per-cell sweep.

Sweeps JANUS 011_01 packet-error rate as a function of range across
multiple Wenz sea states. Each cell runs the simulator at one
(range, sea_state, seed) tuple with both modems stationary, sends N
JANUS messages, scores PER, then tears down. One simulator process per
range value because the geometric scene resets R to ``initial_range_m``
on every TX, so a within-process closing-range sweep would transmit
every packet at R0.

Outputs (under ``--out``, default ``experiments/results/exp3/``):

  <cell>/scenario.yaml
  <cell>/<scenario>_summary.json
  <cell>/modem_a_cdc.log, modem_b_cdc.log
  <cell>/modem_a_events.jsonl
  <cell>/tx_log.csv             host-driver TX requests
  <cell>/cell_meta.json         {range_m, sea_state, seed, ...}

  per_vs_range.csv              long-form: range_m, sea_state, seed,
                                n_total, n_received, per
  per_vs_range_pooled.csv       pooled across seeds with Wilson 95% CI
  processing_table.csv          per-cell processing-time table
  fig_janus_per.pdf             PER vs range, one curve per sea state

Modems must be pre-configured in JANUS mode out of band; the driver does
not flip the firmware protocol parameter.
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
from typing import Callable, Sequence

from experiments.lib import plotting
from experiments.lib.cdc_console import CdcConsole
from experiments.lib.runner import CellHandle, Sweep


REPO        = Path(__file__).resolve().parents[1]
CONFIG_DIR  = REPO / "experiments" / "configs" / "exp3"
TEMPLATE    = CONFIG_DIR / "exp3_janus_per.yaml.j2"

# Geometric scene parameters that mirror the YAML template; used here only
# for figure annotations.
SOUND_SPEED_M_S     = 1500.0
WATER_DEPTH_M       = 120.0
SOURCE_DEPTH_M      =  50.0
RECEIVER_DEPTH_M    = 100.0

# Band E JANUS: 250 symbols/sec, ~4 ms symbol period.
JANUS_BAUD            = 250.0
JANUS_SYMBOL_PERIOD_S = 1.0 / JANUS_BAUD

# Default sweep grid.
DEFAULT_RANGES_M    = (200.0, 300.0, 400.0, 500.0,
                       600.0, 700.0, 800.0, 900.0, 1000.0)
DEFAULT_SEA_STATES  = (1, 3, 5)
DEFAULT_SEEDS       = 5
DEFAULT_PACKETS_PER_CELL = 15

# Closing velocity (negative = approaching). At v=-1 m/s the within-message
# symbol-clock drift is enough to produce graded cargo decode failures
# while still letting most packets sync. Pure-static (v=0) gives binary
# PER curves concentrated at the SNR sync threshold.
DEFAULT_CLOSING_VELOCITY_M_S = -1.0

DEFAULT_CADENCE_S            = 5.0   # firmware-limited minimum
DEFAULT_FIRST_REQUEST_DELAY_S = 5.0
DEFAULT_RX_GRACE_S           = 8.0   # wait after last TX for late RX prints

# Per-cell wall-clock ceiling. Bounds a wedged cell without retrying
# indefinitely.
DEFAULT_MAX_CELL_RUNTIME_S = 300.0
SIGTERM_GRACE_S = 6.0

SMS_PROBE_RE  = re.compile(r"SMS:\s*PROBE\s+(\d+)")
READY_PATTERN = re.compile(r"Simulation running")


# ---------------------------------------------------------------------------
# Pure analysis primitives
# ---------------------------------------------------------------------------

@dataclass
class TxRequest:
    request_id:    int          # probe sequence number embedded in the SMS
    request_ts_ns: int          # host monotonic_ns at send_janus_011_01_tx call
    payload:       str          # the SMS text actually transmitted


@dataclass
class CellResult:
    """Per-cell PER summary. One cell = one (range, sea_state, seed) tuple."""
    range_m:    float
    sea_state:  int
    seed:       int
    n_total:    int
    n_received: int

    @property
    def per(self) -> float:
        if self.n_total <= 0:
            return float("nan")
        return 1.0 - self.n_received / self.n_total


@dataclass
class PooledPoint:
    """Pooled (range, sea_state) PER + Wilson 95% CI."""
    range_m:    float
    sea_state:  int
    n_total:    int
    n_received: int
    per:        float           # NaN when n_total == 0
    ci_lo:      float           # NaN when n_total == 0
    ci_hi:      float


def parse_sms_seq(line: str) -> int | None:
    """Extract the PROBE sequence number from a firmware ``SMS: PROBE NNN``
    line, or ``None`` if the line isn't a PROBE payload.
    """
    if not isinstance(line, str):
        return None
    m = SMS_PROBE_RE.search(line)
    if m is None:
        return None
    try:
        return int(m.group(1))
    except ValueError:
        return None


def wilson_ci(n_failures: int,
              n_total:    int,
              z:          float = 1.96) -> tuple[float, float]:
    """Wilson score 95% confidence interval for a binomial proportion.

    Returns ``(lo, hi)`` such that the true PER is in ``[lo, hi]`` with
    ~95% confidence; preferred over the Wald interval because PER spends
    most of its time near 0 or 1 where Wald misbehaves. Returns
    ``(NaN, NaN)`` for empty bins.
    """
    if n_total <= 0:
        return (float("nan"), float("nan"))
    p = n_failures / n_total
    denom  = 1.0 + (z * z) / n_total
    center = (p + (z * z) / (2.0 * n_total)) / denom
    margin = (z * math.sqrt(p * (1.0 - p) / n_total
                            + (z * z) / (4.0 * n_total * n_total))) / denom
    return (max(0.0, center - margin), min(1.0, center + margin))


def pool_across_seeds(cells: Sequence[CellResult]) -> list[PooledPoint]:
    """Group cells by (range, sea_state), sum n_total / n_received, and
    compute the Wilson CI on the pooled count. Pooled counts are the
    correct estimator for an unequal-n binomial mean — better than
    averaging per-seed PERs.
    """
    buckets: dict[tuple[float, int], list[int]] = {}
    for c in cells:
        key = (float(c.range_m), int(c.sea_state))
        b = buckets.setdefault(key, [0, 0])
        b[0] += c.n_total
        b[1] += c.n_received
    out: list[PooledPoint] = []
    for (r, ss), (tot, rcv) in sorted(buckets.items()):
        if tot == 0:
            out.append(PooledPoint(r, ss, 0, 0,
                                   float("nan"), float("nan"), float("nan")))
            continue
        per = 1.0 - rcv / tot
        lo, hi = wilson_ci(tot - rcv, tot)
        out.append(PooledPoint(r, ss, tot, rcv, per, lo, hi))
    return out


# ---------------------------------------------------------------------------
# Loaders for per-cell artifacts
# ---------------------------------------------------------------------------

def load_tx_log(cell_dir: Path) -> list[TxRequest]:
    """Parse the host-driver-maintained tx_log.csv."""
    path = cell_dir / "tx_log.csv"
    out: list[TxRequest] = []
    if not path.is_file():
        return out
    with path.open() as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            try:
                out.append(TxRequest(
                    request_id    = int(row["request_id"]),
                    request_ts_ns = int(row["request_ts_ns"]),
                    payload       = row["payload"],
                ))
            except (KeyError, ValueError):
                continue
    return out


def load_received_seqs(cell_dir: Path) -> set[int]:
    """Scan modem_b_cdc.log for ``SMS: PROBE NNN`` lines."""
    path = cell_dir / "modem_b_cdc.log"
    if not path.is_file():
        return set()
    out: set[int] = set()
    for line in path.read_text().splitlines():
        seq = parse_sms_seq(line)
        if seq is not None:
            out.add(seq)
    return out


def load_cell_meta(cell_dir: Path) -> dict:
    path = cell_dir / "cell_meta.json"
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return {}


def load_summary(cell_dir: Path) -> dict | None:
    candidates = sorted(cell_dir.glob("*_summary.json"))
    if not candidates:
        return None
    try:
        return json.loads(candidates[0].read_text())
    except json.JSONDecodeError:
        return None


def _cell_is_valid(cell_dir: Path) -> bool:
    """Heuristic check that a finished cell is usable for analysis. Catches
    silent-failure modes (CDC reader died mid-cell, modem TTY couldn't be
    opened, simulator wedged before any TX, watchdog killed the cell during
    init) without parsing the full artifacts. Requires tx_log.csv to have
    at least one request row and modem_b_cdc.log to be present and at
    least 100 bytes.
    """
    tx_log  = cell_dir / "tx_log.csv"
    cdc_log = cell_dir / "modem_b_cdc.log"
    if not tx_log.is_file():
        return False
    try:
        with tx_log.open() as fp:
            rows = sum(1 for _ in fp)
    except OSError:
        return False
    if rows < 2:                # header + at least one request
        return False
    if not cdc_log.is_file():
        return False
    try:
        if cdc_log.stat().st_size < 100:
            return False
    except OSError:
        return False
    return True


# ---------------------------------------------------------------------------
# Per-cell analysis
# ---------------------------------------------------------------------------

@dataclass
class CellAnalysis:
    cell_dir:   Path
    result:     CellResult
    summary:    dict | None


def analyse_cell(cell_dir: Path) -> CellAnalysis | None:
    """Materialise the per-cell analysis from artifacts in ``cell_dir``.
    Returns ``None`` if the cell lacks tx_log.csv or cell_meta.json (e.g.
    a cell that aborted before pre_run wrote any host-side artifacts).
    """
    meta     = load_cell_meta(cell_dir)
    requests = load_tx_log(cell_dir)
    if not requests:
        return None
    if "range_m" not in meta:
        return None
    received = load_received_seqs(cell_dir)
    summary  = load_summary(cell_dir)

    # Loud warning for the silent-failure mode that _cell_is_valid
    # catches.
    cdc_path = cell_dir / "modem_b_cdc.log"
    cdc_size = cdc_path.stat().st_size if cdc_path.is_file() else 0
    if requests and not received and cdc_size < 100:
        sys.stderr.write(
            f"[exp3] WARNING: {cell_dir.name}: {len(requests)} TX requests "
            f"but modem_b_cdc.log is {'missing' if cdc_size == 0 else 'tiny'} "
            f"({cdc_size} bytes) — the CDC console likely failed to attach. "
            "Check that no other terminal program (picocom/screen/IDE serial "
            "monitor) is holding /dev/ttyACM*. PER for this cell will read "
            "as 1.0 regardless of actual on-air decode success.\n")

    requested_ids = {r.request_id for r in requests}
    n_received    = len(requested_ids & received)
    result = CellResult(
        range_m    = float(meta["range_m"]),
        sea_state  = int(meta.get("sea_state", -1)),
        seed       = int(meta.get("seed",      -1)),
        n_total    = len(requests),
        n_received = n_received,
    )
    return CellAnalysis(cell_dir=cell_dir, result=result, summary=summary)


def analyse(out_dir: Path) -> list[CellAnalysis]:
    """Walk ``out_dir`` for every cell with a tx_log.csv + cell_meta.json."""
    out: list[CellAnalysis] = []
    for child in sorted(out_dir.iterdir()):
        if not child.is_dir():
            continue
        a = analyse_cell(child)
        if a is not None:
            out.append(a)
    return out


# ---------------------------------------------------------------------------
# CSV writers
# ---------------------------------------------------------------------------

def write_per_vs_range_csv(analyses: Sequence[CellAnalysis],
                           path:     Path) -> None:
    """Long-form per-cell PER. One row per (range, sea_state, seed)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = sorted((a.result for a in analyses),
                  key=lambda r: (r.range_m, r.sea_state, r.seed))
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["range_m", "sea_state", "seed",
                    "n_total", "n_received", "per"])
        for r in rows:
            w.writerow([
                f"{r.range_m:.1f}",
                r.sea_state,
                r.seed,
                r.n_total,
                r.n_received,
                ("" if math.isnan(r.per) else f"{r.per:.6f}"),
            ])


def write_pooled_per_csv(analyses: Sequence[CellAnalysis],
                         path:     Path) -> None:
    """Pooled-across-seeds PER per (range, sea_state) + Wilson 95% CI. One
    row per (range, sea_state), with the count pooled across seeds so the
    Wilson interval reflects the full sample size.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    pooled = pool_across_seeds([a.result for a in analyses])
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["range_m", "sea_state",
                    "n_total", "n_received",
                    "per", "per_ci_lo", "per_ci_hi"])
        for p in pooled:
            if math.isnan(p.per):
                w.writerow([f"{p.range_m:.1f}", p.sea_state,
                            p.n_total, p.n_received, "", "", ""])
            else:
                w.writerow([
                    f"{p.range_m:.1f}",
                    p.sea_state,
                    p.n_total,
                    p.n_received,
                    f"{p.per:.6f}",
                    f"{p.ci_lo:.6f}",
                    f"{p.ci_hi:.6f}",
                ])


def write_processing_table_csv(analyses: Sequence[CellAnalysis],
                               path:     Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["range_m", "sea_state", "seed",
                    "mean_us", "p99_us", "max_us", "underruns"])
        for a in analyses:
            pt = (a.summary or {}).get("processing_time", {}) or {}
            w.writerow([
                f"{a.result.range_m:.1f}",
                a.result.sea_state,
                a.result.seed,
                pt.get("mean_us",       ""),
                pt.get("p99_us",        ""),
                pt.get("max_us",        ""),
                pt.get("underrun_count", ""),
            ])


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

def _surface_path_threshold_range_m(symbol_period_s: float) -> float | None:
    """Range at which the surface-bounce excess delay equals one JANUS
    symbol period. Returns ``None`` if the threshold is unsolvable.
    """
    return _path_threshold_range_m(
        h_direct_m  = abs(RECEIVER_DEPTH_M - SOURCE_DEPTH_M),
        h_bounce_m  = SOURCE_DEPTH_M + RECEIVER_DEPTH_M,
        excess_m    = symbol_period_s * SOUND_SPEED_M_S,
    )


def _bottom_path_threshold_range_m(symbol_period_s: float) -> float | None:
    """Range at which the bottom-bounce excess delay equals one JANUS
    symbol period."""
    return _path_threshold_range_m(
        h_direct_m  = abs(RECEIVER_DEPTH_M - SOURCE_DEPTH_M),
        h_bounce_m  = (WATER_DEPTH_M - SOURCE_DEPTH_M) + (WATER_DEPTH_M - RECEIVER_DEPTH_M),
        excess_m    = symbol_period_s * SOUND_SPEED_M_S,
    )


def _path_threshold_range_m(h_direct_m: float,
                            h_bounce_m: float,
                            excess_m:   float) -> float | None:
    """Solve sqrt(R^2 + h_b^2) - sqrt(R^2 + h_d^2) = excess for R >= 0."""
    if excess_m <= 0:
        return None
    num = h_bounce_m ** 2 - h_direct_m ** 2 - excess_m ** 2
    if num <= 0:
        return None
    sqrt_u = num / (2.0 * excess_m)
    u = sqrt_u ** 2
    R2 = u - h_direct_m ** 2
    if R2 < 0:
        return None
    return float(math.sqrt(R2))


def render_per_figure(analyses:   Sequence[CellAnalysis],
                      savepath:   Path,
                      sea_states: Sequence[int] = DEFAULT_SEA_STATES) -> None:
    """PER-vs-range curves, one per sea state."""
    import matplotlib.pyplot as plt           # local: plotting is optional
    plotting.apply_paper_style()
    fig, ax = plt.subplots()

    pooled = pool_across_seeds([a.result for a in analyses])
    if not pooled:
        savepath.parent.mkdir(parents=True, exist_ok=True)
        plotting.save_figure(fig, savepath)
        return

    xs_all = [p.range_m for p in pooled]
    x_lo   = min(xs_all)
    x_hi   = max(xs_all)
    if x_lo == x_hi:                # single range: pad axis so dot is visible
        x_lo -= 50.0
        x_hi += 50.0

    has_data = False
    for ss in sea_states:
        pts = [p for p in pooled
               if p.sea_state == ss and not math.isnan(p.per)]
        if not pts:
            continue
        has_data = True
        pts.sort(key=lambda p: p.range_m)
        xs  = [p.range_m for p in pts]
        ys  = [p.per     for p in pts]
        los = [p.ci_lo   for p in pts]
        his = [p.ci_hi   for p in pts]
        line, = ax.plot(xs, ys, marker="o", label=f"sea state {ss}")
        ax.fill_between(xs, los, his,
                        alpha=0.18, color=line.get_color(),
                        linewidth=0)

    ax.set_xlabel("Range (m)")
    ax.set_ylabel("Packet error rate")
    ax.set_ylim(-0.02, 1.05)
    ax.set_xlim(x_lo, x_hi)
    ax.grid(True, alpha=0.3)
    if has_data:
        ax.legend(loc="best", frameon=False)
    savepath.parent.mkdir(parents=True, exist_ok=True)
    plotting.save_figure(fig, savepath)


# ---------------------------------------------------------------------------
# CDC driver — per-cell pre/post hooks
# ---------------------------------------------------------------------------

class _JanusDriver:
    """Owns the CDC consoles and the TX-loop thread for one cell."""

    def __init__(self,
                 modem_a_serial:        str,
                 modem_b_serial:        str,
                 *,
                 range_m:               float,
                 sea_state:             int,
                 seed:                  int,
                 velocity_m_s:          float,
                 cadence_s:             float,
                 first_request_delay_s: float,
                 packets_per_cell:      int,
                 rx_grace_s:            float,
                 cdc_factory:           Callable[..., CdcConsole] = CdcConsole.attach,
                 ready_timeout_s:       float = 15.0) -> None:
        self.modem_a_serial         = modem_a_serial
        self.modem_b_serial         = modem_b_serial
        self.range_m                = float(range_m)
        self.sea_state              = int(sea_state)
        self.seed                   = int(seed)
        self.velocity_m_s           = float(velocity_m_s)
        self.cadence_s              = float(cadence_s)
        self.first_request_delay_s  = float(first_request_delay_s)
        self.packets_per_cell       = int(packets_per_cell)
        self.rx_grace_s             = float(rx_grace_s)
        self._cdc_factory           = cdc_factory
        self._ready_timeout         = ready_timeout_s

        self._consoles:  dict[str, list[CdcConsole]]     = {}
        self._tx_logs:   dict[str, list[TxRequest]]      = {}
        self._spawns:    dict[str, int]                  = {}
        self._done_evts: dict[str, threading.Event]      = {}
        self._stop_evts: dict[str, threading.Event]      = {}
        self._threads:   dict[str, threading.Thread]     = {}

    # ---- Sweep hooks ---------------------------------------------------

    def pre_run(self, handle: CellHandle) -> None:
        spawn_ns = time.monotonic_ns()
        self._spawns[handle.cell_id]  = spawn_ns
        self._tx_logs[handle.cell_id] = []
        self._wait_for_ready(handle.cell_dir / "stdout.log")
        cdc_a = self._cdc_factory(
            modem_id   = "modem_a",
            usb_serial = self.modem_a_serial,
            log_path   = handle.cell_dir / "modem_a_cdc.log",
        )
        cdc_b = self._cdc_factory(
            modem_id   = "modem_b",
            usb_serial = self.modem_b_serial,
            log_path   = handle.cell_dir / "modem_b_cdc.log",
        )
        self._consoles[handle.cell_id] = [cdc_a, cdc_b]

        # Ping each modem and require a Main Menu echo before counting the
        # cell as ready. Distinguishes "CDC attached but TTY silent" from a
        # genuine PER=1.0 result.
        try:
            for cdc in (cdc_a, cdc_b):
                if not cdc.verify_main_menu(timeout_s=2.0):
                    raise RuntimeError(
                        f"{cdc.modem_id}: CDC attached but no 'Main Menu' "
                        "echo within 2.0 s — modem unresponsive or wedged.")
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
            target = self._tx_loop,
            args   = (handle.cell_id, cdc_a, cdc_b, done, stop),
            name   = f"exp3-tx-{handle.cell_id[:8]}",
            daemon = True,
        )
        self._threads[handle.cell_id] = worker
        worker.start()

    def post_run(self, handle: CellHandle) -> None:
        stop = self._stop_evts.pop(handle.cell_id, None)
        if stop is not None:
            stop.set()
        thread = self._threads.pop(handle.cell_id, None)
        if thread is not None:
            thread.join(timeout=5.0)

        # Flush tx_log.csv and cell_meta.json before detaching CDCs so
        # late-arriving RX prints still land in modem_b_cdc.log.
        tx_log = self._tx_logs.pop(handle.cell_id, [])
        self._write_tx_log(handle.cell_dir, tx_log)
        self._write_cell_meta(handle.cell_dir,
                              spawn_ns=self._spawns.pop(handle.cell_id, 0))

        # Beat for modem B to print any in-flight decode block.
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
            "[exp3] simulator did not print 'Simulation running' within "
            f"{self._ready_timeout:.1f}s — proceeding anyway; first few "
            "TXs may be lost to startup.\n")

    def _tx_loop(self,
                 cell_id:  str,
                 cdc_a:    CdcConsole,
                 cdc_b:    CdcConsole,
                 done:     threading.Event,
                 stop:     threading.Event) -> None:
        tx_log = self._tx_logs[cell_id]
        if self.first_request_delay_s > 0.0:
            stop.wait(self.first_request_delay_s)
        seq = 0
        while seq < self.packets_per_cell and not stop.is_set():
            # Bail if either CDC reader thread died mid-cell. A dead
            # modem-B reader means RX prints don't reach the log and
            # decoded packets are scored as lost; abort and let the sweep
            # wrapper retry.
            if not cdc_a.is_reader_alive() or not cdc_b.is_reader_alive():
                which = "A" if not cdc_a.is_reader_alive() else "B"
                sys.stderr.write(f"[exp3] cell={cell_id[:8]} seq={seq}: "
                                 f"modem-{which} CDC reader died — aborting cell "
                                 "to avoid corrupted PER measurements.\n")
                break
            payload = f"PROBE {seq:03d}"
            now_ns  = time.monotonic_ns()
            try:
                cdc_a.send_janus_011_01_tx(payload)
            except Exception as exc:                            # noqa: BLE001
                sys.stderr.write(f"[exp3] cell={cell_id[:8]} seq={seq} "
                                 f"TX failed: {exc}\n")
                # If the TX-side TTY went away the next loop's
                # is_reader_alive() check bails.
            tx_log.append(TxRequest(
                request_id    = seq,
                request_ts_ns = now_ns,
                payload       = payload,
            ))
            seq += 1
            stop.wait(self.cadence_s)
        done.set()

    def _write_tx_log(self, cell_dir: Path, log: list[TxRequest]) -> None:
        path = cell_dir / "tx_log.csv"
        with path.open("w", newline="") as fp:
            w = csv.writer(fp)
            w.writerow(["request_id", "request_ts_ns", "payload"])
            for r in log:
                w.writerow([r.request_id, r.request_ts_ns, r.payload])

    def _write_cell_meta(self, cell_dir: Path, *, spawn_ns: int) -> None:
        meta = dict(
            simulator_spawn_ns = int(spawn_ns),
            range_m            = self.range_m,
            velocity_m_s       = self.velocity_m_s,
            sea_state          = self.sea_state,
            seed               = self.seed,
            cadence_s          = self.cadence_s,
            packets_per_cell   = self.packets_per_cell,
        )
        (cell_dir / "cell_meta.json").write_text(json.dumps(meta, indent=2))


# ---------------------------------------------------------------------------
# Sweep wiring
# ---------------------------------------------------------------------------

def _per_cell_sweep(*,
                    range_m:               float,
                    sea_state:             int,
                    seed:                  int,
                    velocity_m_s:          float,
                    out_root:              Path,
                    binary:                Path | str,
                    modem_a_serial:        str,
                    modem_b_serial:        str,
                    cadence_s:             float,
                    first_request_delay_s: float,
                    max_cell_runtime_s:    float,
                    rx_grace_s:            float,
                    log_raw_rx:            bool,
                    retries:               int,
                    packets_per_cell:      int) -> None:
    """Run one (range, sea_state, seed) cell as a one-cell Sweep."""
    driver = _JanusDriver(
        modem_a_serial         = modem_a_serial,
        modem_b_serial         = modem_b_serial,
        range_m                = range_m,
        sea_state              = sea_state,
        seed                   = seed,
        velocity_m_s           = velocity_m_s,
        cadence_s              = cadence_s,
        first_request_delay_s  = first_request_delay_s,
        packets_per_cell       = packets_per_cell,
        rx_grace_s             = rx_grace_s,
    )
    sweep = Sweep(
        template_path     = TEMPLATE,
        parameters        = {
            "range_m":   [range_m],
            "sea_state": [sea_state],
            "seed":      [seed],
        },
        extra_params      = dict(
            modem_a_serial = modem_a_serial,
            modem_b_serial = modem_b_serial,
            log_raw_rx     = bool(log_raw_rx),
            velocity_m_s   = velocity_m_s,
        ),
        binary             = binary,
        out_dir            = out_root,
        duration_s         = float("inf"),       # cell_done drives shutdown
        parallel           = 1,
        sigterm_grace_s    = SIGTERM_GRACE_S,
        retries            = retries,
        max_cell_runtime_s = max_cell_runtime_s,
        pre_run            = driver.pre_run,
        post_run           = driver.post_run,
        stop_condition     = driver.cell_done,
    )
    sweep.run()


RETRY_BACKOFF_S = 3.0


def _per_cell_sweep_with_retry(**kwargs) -> None:
    """Run one (range, sea_state, seed) cell, retrying indefinitely until
    the simulator produces a valid result (``_cell_is_valid`` returns
    True). Triggers another attempt on:

      * sweep.run() raising (simulator crash, USB enumeration failure,
        missing binary, ...);
      * cell directory created but no TX logged (init wedge);
      * CDC console failed to attach to modem B (silent PER=1.0).

    Failed attempts are renamed ``<cell>_failed_attempt<N>`` so ``analyse``
    skips them. Intended for unattended overnight operation; Ctrl+C
    aborts.
    """
    out_root:  Path = kwargs["out_root"]
    range_m         = kwargs["range_m"]
    sea_state       = kwargs["sea_state"]
    seed            = kwargs["seed"]

    attempt = 0
    while True:
        attempt += 1
        cells_before = {p.name for p in out_root.iterdir() if p.is_dir()}

        sweep_exc: BaseException | None = None
        try:
            _per_cell_sweep(**kwargs)
        except KeyboardInterrupt:
            raise
        except BaseException as exc:                    # noqa: BLE001
            sweep_exc = exc
            sys.stderr.write(
                f"[exp3] R={range_m:.0f}m ss={sea_state} seed={seed} "
                f"attempt={attempt}: sweep raised {type(exc).__name__}: {exc}\n")

        new_cells = sorted(
            out_root / name for name in
            ({p.name for p in out_root.iterdir() if p.is_dir()} - cells_before))
        valid_cells = [c for c in new_cells if _cell_is_valid(c)]

        if sweep_exc is None and valid_cells:
            if attempt > 1:
                sys.stderr.write(
                    f"[exp3] R={range_m:.0f}m ss={sea_state} seed={seed}: "
                    f"succeeded on attempt {attempt}.\n")
            return

        for c in new_cells:
            if c in valid_cells:
                continue
            quarantine = c.with_name(f"{c.name}_failed_attempt{attempt}")
            try:
                c.rename(quarantine)
                sys.stderr.write(
                    f"[exp3]   quarantined {c.name} → {quarantine.name}\n")
            except OSError as e:
                sys.stderr.write(
                    f"[exp3]   could not quarantine {c.name}: {e}\n")

        if sweep_exc is None:
            sys.stderr.write(
                f"[exp3] R={range_m:.0f}m ss={sea_state} seed={seed} "
                f"attempt={attempt}: no usable cell produced (silent CDC "
                "failure, wedged modem, or watchdog-killed cell) — retrying.\n")
        time.sleep(RETRY_BACKOFF_S)


def _print_summary(analyses: Sequence[CellAnalysis]) -> None:
    pooled = pool_across_seeds([a.result for a in analyses])
    by_state: dict[int, list[PooledPoint]] = {}
    for p in pooled:
        by_state.setdefault(p.sea_state, []).append(p)
    sys.stderr.write("[exp3] PER vs range (pooled across seeds, Wilson 95% CI):\n")
    for ss in sorted(by_state):
        points = sorted(by_state[ss], key=lambda p: p.range_m)
        sys.stderr.write(f"  sea state {ss}:\n")
        for p in points:
            if math.isnan(p.per):
                sys.stderr.write(f"     R={p.range_m:7.1f} m   "
                                 f"PER=  nan                 n={p.n_total:4d}\n")
            else:
                sys.stderr.write(
                    f"     R={p.range_m:7.1f} m   "
                    f"PER={p.per:5.3f} "
                    f"[{p.ci_lo:5.3f}, {p.ci_hi:5.3f}]   "
                    f"n={p.n_total:4d}\n")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", default="experiments/results/exp3",
                   help="output directory for sweep artifacts")
    p.add_argument("--binary",
                   default=str(REPO / "build" / "openCREST"),
                   help="path to the openCREST simulator binary")
    p.add_argument("--ranges", default=",".join(f"{r:g}"
                                                for r in DEFAULT_RANGES_M),
                   help="comma-separated range values in metres "
                        f"(default {','.join(f'{r:g}' for r in DEFAULT_RANGES_M)}). "
                        "One simulator process per range.")
    p.add_argument("--sea-states", default=",".join(str(s)
                                                    for s in DEFAULT_SEA_STATES),
                   help="comma-separated Wenz sea state indices "
                        f"(default {','.join(str(s) for s in DEFAULT_SEA_STATES)})")
    p.add_argument("--seeds", type=int, default=DEFAULT_SEEDS,
                   help=f"number of seeds per (range, sea_state) (default "
                        f"{DEFAULT_SEEDS}).")
    p.add_argument("--packets-per-cell", type=int,
                   default=DEFAULT_PACKETS_PER_CELL,
                   help=f"number of JANUS messages per cell "
                        f"(default {DEFAULT_PACKETS_PER_CELL})")
    p.add_argument("--velocity-m-s", type=float,
                   default=DEFAULT_CLOSING_VELOCITY_M_S,
                   help="closing velocity of modem A in m/s (negative = "
                        f"approaching, default {DEFAULT_CLOSING_VELOCITY_M_S}). "
                        "Range resets to initial_range_m on each TX so "
                        "velocity affects only within-message Doppler / "
                        "multipath time-evolution, not the starting range.")
    p.add_argument("--cadence-s", type=float, default=DEFAULT_CADENCE_S,
                   help="seconds between JANUS TX requests on modem A "
                        f"(firmware-limited minimum {DEFAULT_CADENCE_S})")
    p.add_argument("--first-request-delay-s", type=float,
                   default=DEFAULT_FIRST_REQUEST_DELAY_S,
                   help="settle time after 'Simulation running' before the "
                        f"first TX (default {DEFAULT_FIRST_REQUEST_DELAY_S})")
    p.add_argument("--rx-grace-s", type=float, default=DEFAULT_RX_GRACE_S,
                   help="wait this long after the last TX before tearing the "
                        f"CDCs down, so trailing RX prints land in the log "
                        f"(default {DEFAULT_RX_GRACE_S})")
    p.add_argument("--max-cell-runtime-s", type=float,
                   default=DEFAULT_MAX_CELL_RUNTIME_S,
                   help=f"watchdog ceiling per cell (default {DEFAULT_MAX_CELL_RUNTIME_S})")
    p.add_argument("--modem-a-serial", default="OA-2-1")
    p.add_argument("--modem-b-serial", default="OA-2-2")
    p.add_argument("--retries", type=int, default=0)
    p.add_argument("--log-raw-rx", action="store_true",
                   help="enable raw-RX WAV logging (off by default; PER "
                        "analysis doesn't need it and WAVs balloon disk usage)")
    p.add_argument("--analysis-only", action="store_true",
                   help="don't run the sweep — only post-process artifacts "
                        "already present in --out")
    args = p.parse_args(argv)

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    ranges     = [float(r) for r in args.ranges.split(",") if r.strip()]
    sea_states = [int(s)   for s in args.sea_states.split(",") if s.strip()]
    seeds      = list(range(int(args.seeds)))

    if not args.analysis_only:
        for range_m in ranges:
            for ss in sea_states:
                for seed in seeds:
                    _per_cell_sweep_with_retry(
                        range_m               = range_m,
                        sea_state             = ss,
                        seed                  = seed,
                        velocity_m_s          = args.velocity_m_s,
                        out_root              = out_dir,
                        binary                = args.binary,
                        modem_a_serial        = args.modem_a_serial,
                        modem_b_serial        = args.modem_b_serial,
                        cadence_s             = args.cadence_s,
                        first_request_delay_s = args.first_request_delay_s,
                        max_cell_runtime_s    = args.max_cell_runtime_s,
                        rx_grace_s            = args.rx_grace_s,
                        log_raw_rx            = args.log_raw_rx,
                        retries               = args.retries,
                        packets_per_cell      = args.packets_per_cell,
                    )

    analyses = analyse(out_dir)
    if not analyses:
        sys.stderr.write("[exp3] no analysable cells found in "
                         f"{out_dir} — aborting\n")
        return 3

    write_per_vs_range_csv     (analyses, out_dir / "per_vs_range.csv")
    write_pooled_per_csv       (analyses, out_dir / "per_vs_range_pooled.csv")
    write_processing_table_csv (analyses, out_dir / "processing_table.csv")
    render_per_figure          (analyses, out_dir / "fig_janus_per.pdf",
                                sea_states=sea_states)
    _print_summary(analyses)
    return 0


if __name__ == "__main__":
    sys.exit(main())
