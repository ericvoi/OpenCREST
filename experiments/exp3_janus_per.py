"""Experiment 3 — JANUS PER vs range, sea-state sweep (paper §4.4).

Reproduces Fig. 4: JANUS 011_01 packet-error rate as a function of
instantaneous range during a 1000 m → 200 m approach at v=-2 m/s,
swept over Wenz sea states {1, 3, 5}. Each cell runs the simulator
once with the geometric scene from Exp 1 (D=120 m, z_s=50 m, z_r=100 m,
three macro paths). The host driver issues a JANUS 011_01 SMS once
every ``--cadence-s`` seconds on modem A; modem B's CDC console prints
a multi-line decode block for every JANUS frame that decodes. PER
per range bin is 1 − (received / total).

Outputs (under ``--out``, default ``experiments/results/exp3/``):

  <cell>/scenario.yaml
  <cell>/<scenario>_summary.json
  <cell>/modem_a_cdc.log, modem_b_cdc.log
  <cell>/modem_a_events.jsonl
  <cell>/tx_log.csv             host-driver TX requests
  <cell>/cell_meta.json         {simulator_spawn_ns, r0_m, v_m_s, ...}

  per_vs_range.csv              long-form: sea_state, seed, range_bin_m, n_total, n_received, per
  processing_table.csv          sustained-throughput sanity per cell
  fig_janus_per.pdf             paper Fig. 4

The simulator binary stays test-agnostic (per
``feedback_opencrest_no_test_specific_code.md``); all JANUS-specific
behaviour lives here. Modems are pre-configured in JANUS mode out of
band — the driver does not flip the firmware protocol parameter.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

import numpy as np

from experiments.lib import plotting
from experiments.lib.cdc_console import CdcConsole
from experiments.lib.runner import CellHandle, Sweep


REPO        = Path(__file__).resolve().parents[1]
CONFIG_DIR  = REPO / "experiments" / "configs" / "exp3"
TEMPLATE    = CONFIG_DIR / "exp3_janus_per.yaml.j2"

# Geometric scene parameters that must match the YAML template. The driver
# uses these to compute range-at-TX and the multipath-delay vertical
# annotations on the figure; they are *not* the source of truth for the
# simulator (the YAML is). If the template changes these values, update
# the constants here.
INITIAL_RANGE_M     = 1000.0
RANGE_FLOOR_M       =  200.0
# Default closing velocity. Reduced from the paper's -2.0 m/s because the
# OpenAquatix firmware demodulates cargo symbols on a fixed timing grid
# derived from preamble sync — there is no per-symbol re-sync — and v=-2
# produces ~1/3..1/2 a symbol of window drift over a single 250–350-symbol
# cargo at the baud rate used here. v=-1 keeps the drift under 1/4 of a
# symbol, which is inside the cargo demod's tolerance.
DEFAULT_CLOSING_VELOCITY_M_S = -1.0
SOUND_SPEED_M_S     = 1500.0
WATER_DEPTH_M       = 120.0
SOURCE_DEPTH_M      =  50.0
RECEIVER_DEPTH_M    = 100.0

# Band E JANUS: 250 symbols/sec → ~4 ms symbol period.
JANUS_BAUD          = 250.0
JANUS_SYMBOL_PERIOD_S = 1.0 / JANUS_BAUD

DEFAULT_SEA_STATES  = (1, 3, 5)
DEFAULT_SEEDS       = 5
DEFAULT_CADENCE_S   = 5.0          # firmware-limited minimum
DEFAULT_FIRST_REQUEST_DELAY_S = 5.0
DEFAULT_RX_GRACE_S  = 8.0          # extra wait after last TX for late RX prints

# 100 m bins from 200 m to 1000 m → 8 bins.
DEFAULT_BIN_EDGES_M = [200.0, 300.0, 400.0, 500.0,
                       600.0, 700.0, 800.0, 900.0, 1000.0]

DEFAULT_MAX_CELL_RUNTIME_S = 1200.0    # 20 min wall-cap per cell;
                                       # at v=-1 the natural cell is ~13.5 min
                                       # so this leaves ~6 min of headroom for
                                       # init/grace, and a wedged cell costs
                                       # only ~7 min over baseline (vs the
                                       # previous 17 min). Bump back up if you
                                       # drop velocity to -0.5.
SIGTERM_GRACE_S = 6.0

SMS_PROBE_RE  = re.compile(r"SMS:\s*PROBE\s+(\d+)")
READY_PATTERN = re.compile(r"Simulation running")


# ---------------------------------------------------------------------------
# Pure analysis primitives (target for TDD tests)
# ---------------------------------------------------------------------------

@dataclass
class TxRequest:
    request_id:    int          # probe sequence number embedded in the SMS
    request_ts_ns: int          # host monotonic_ns at send_janus_011_01_tx call
    payload:       str          # the SMS text actually transmitted


@dataclass
class TxEvent:
    sequence_id:  int           # firmware-emitted monotonic TX counter
    start_ns:     int
    end_ns:       int
    sample_count: int


@dataclass
class TxRecord:
    request_id:        int
    payload:           str
    tx_event_start_ns: int
    range_at_tx_m:     float
    received:          bool


@dataclass
class PerBin:
    lo_m:       float
    hi_m:       float
    n_total:    int
    n_received: int
    per:        float           # NaN when n_total == 0


def compute_range_at_tx(event_start_ns:      int,
                        simulator_spawn_ns:  int,
                        r0_m:                float,
                        v_m_s:               float) -> float:
    """Closed-form geometric R(t).

    The simulator parametrises the geometric scene by receiver-side sample
    cursor, which in steady state advances at wall-clock rate — so the
    monotonic-ns delta between TX-event start and simulator spawn is a good
    proxy for the scene's internal ``t`` (within the few-second initialisation
    delay that's identical across cells).
    """
    t_s = (event_start_ns - simulator_spawn_ns) / 1e9
    return float(r0_m + v_m_s * t_s)


def parse_sms_seq(line: str) -> int | None:
    """Extract the PROBE sequence number from a firmware decode line, or
    ``None`` if the line isn't a PROBE payload.

    Modem firmware prints a multi-line block on every JANUS RX (see
    ``comm_print.c``); the only line we care about is ``SMS: <payload>``.
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
    ~95% confidence. Used in place of the Wald interval because Wald
    misbehaves when ``p`` is near 0 or 1, exactly where this experiment's
    PER spends most of its time. Returns ``(NaN, NaN)`` for empty bins.
    """
    if n_total <= 0:
        return (float("nan"), float("nan"))
    p = n_failures / n_total
    denom  = 1.0 + (z * z) / n_total
    center = (p + (z * z) / (2.0 * n_total)) / denom
    margin = (z * math.sqrt(p * (1.0 - p) / n_total
                            + (z * z) / (4.0 * n_total * n_total))) / denom
    return (max(0.0, center - margin), min(1.0, center + margin))


def aggregate_per(records: Sequence[TxRecord],
                  bin_edges_m: Sequence[float]) -> list[PerBin]:
    """Bin records by ``range_at_tx_m`` into half-open ``[lo, hi)`` bins.

    Records whose range falls outside the lowest/highest edge are dropped.
    Empty bins report ``per = NaN`` so they don't pollute downstream
    aggregations (e.g. mean across seeds).
    """
    if len(bin_edges_m) < 2:
        return []
    edges = list(bin_edges_m)
    n_bins = len(edges) - 1
    totals = [0] * n_bins
    received = [0] * n_bins
    for r in records:
        rng = r.range_at_tx_m
        if rng < edges[0] or rng >= edges[-1]:
            continue
        # bisect: find the index i s.t. edges[i] <= rng < edges[i+1].
        # Edges are small, linear scan is fine.
        for i in range(n_bins):
            if edges[i] <= rng < edges[i + 1]:
                totals[i] += 1
                if r.received:
                    received[i] += 1
                break
    out: list[PerBin] = []
    for i in range(n_bins):
        n = totals[i]
        per = (1.0 - received[i] / n) if n else float("nan")
        out.append(PerBin(lo_m=edges[i], hi_m=edges[i + 1],
                          n_total=n, n_received=received[i], per=per))
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


def load_modem_a_events(cell_dir: Path) -> list[TxEvent]:
    """Parse modem_a_events.jsonl, returning TX events in start-time order."""
    path = cell_dir / "modem_a_events.jsonl"
    out: list[TxEvent] = []
    if not path.is_file():
        return out
    for raw in path.read_text().splitlines():
        raw = raw.strip()
        if not raw:
            continue
        try:
            doc = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if doc.get("direction") != "tx":
            continue
        try:
            out.append(TxEvent(
                sequence_id  = int(doc.get("sequence_id", -1)),
                start_ns     = int(doc["start_ns"]),
                end_ns       = int(doc["end_ns"]),
                sample_count = int(doc.get("sample_count", 0)),
            ))
        except (KeyError, ValueError):
            continue
    out.sort(key=lambda e: e.start_ns)
    return out


def load_received_seqs(cell_dir: Path) -> set[int]:
    """Scan modem_b_cdc.log for ``SMS: PROBE NNN`` lines.

    The log is the raw CDC-console output (one timestamped line per
    firmware print), so any line containing the SMS payload counts as a
    decode regardless of whether the surrounding decode-block lines made
    it into the queue.
    """
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
    """Load the host-side per-cell metadata (spawn ns, sea state, etc.)."""
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


# ---------------------------------------------------------------------------
# Per-cell analysis
# ---------------------------------------------------------------------------

@dataclass
class CellAnalysis:
    cell_dir:   Path
    sea_state:  int
    seed:       int
    records:    list[TxRecord]
    summary:    dict | None


def _match_requests_to_events(requests: Sequence[TxRequest],
                              events:   Sequence[TxEvent]) -> list[tuple[TxRequest, TxEvent | None]]:
    """Pair each host TX request with the first modem-A TX event whose
    ``start_ns`` is no earlier than the request's ``request_ts_ns`` (allowing
    a small back-slack for clock-edge effects).

    Events are consumed in start-time order; once an event is matched it
    isn't reused. Unmatched requests are paired with ``None``.

    With a 5 s cadence and a non-bursty firmware, this is a clean 1:1 in
    practice; the slack is just defensive.
    """
    out: list[tuple[TxRequest, TxEvent | None]] = []
    ev_iter = iter(events)
    # Negative slack: an event whose start_ns is up to 100 ms BEFORE the
    # request_ts_ns is still considered "this request's event" — the host's
    # clock and the simulator's may have <ms skew, and we'd rather match
    # eagerly than drop legitimate pairings.
    SLACK_NS = 100_000_000
    pending: TxEvent | None = next(ev_iter, None)
    for req in requests:
        while pending is not None and pending.start_ns < req.request_ts_ns - SLACK_NS:
            pending = next(ev_iter, None)
        if pending is None:
            out.append((req, None))
            continue
        out.append((req, pending))
        pending = next(ev_iter, None)
    return out


def build_tx_records(requests:           Sequence[TxRequest],
                     events:             Sequence[TxEvent],
                     received_seqs:      set[int],
                     simulator_spawn_ns: int,
                     r0_m:               float,
                     v_m_s:              float) -> list[TxRecord]:
    pairs = _match_requests_to_events(requests, events)
    records: list[TxRecord] = []
    for req, ev in pairs:
        if ev is None:
            # Request never produced an observable firmware TX — count it as
            # an emitted-but-unobserved packet at the host-time-implied range.
            ev_start = req.request_ts_ns
        else:
            ev_start = ev.start_ns
        rng = compute_range_at_tx(
            event_start_ns      = ev_start,
            simulator_spawn_ns  = simulator_spawn_ns,
            r0_m                = r0_m,
            v_m_s               = v_m_s,
        )
        records.append(TxRecord(
            request_id        = req.request_id,
            payload           = req.payload,
            tx_event_start_ns = ev_start,
            range_at_tx_m     = rng,
            received          = req.request_id in received_seqs,
        ))
    return records


def analyse_cell(cell_dir: Path) -> CellAnalysis | None:
    """Materialise the per-cell analysis from artifacts in ``cell_dir``.

    Returns ``None`` if the cell doesn't have a tx_log.csv (e.g. a Sweep
    cell that aborted before pre_run wrote any host-side artifacts).
    """
    meta     = load_cell_meta(cell_dir)
    requests = load_tx_log(cell_dir)
    if not requests:
        return None
    events     = load_modem_a_events(cell_dir)
    received   = load_received_seqs(cell_dir)
    summary    = load_summary(cell_dir)

    # Loud warning for the classic silent-failure mode: the CDC console
    # couldn't open modem B's TTY (e.g. picocom/screen had it), so the log
    # is empty and every TX gets scored as lost regardless of what actually
    # happened over the air. Without this, you get a clean-looking PER=1.0
    # plot with no hint that the harness never saw any decodes.
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
    spawn_ns   = int(meta.get("simulator_spawn_ns", 0))
    r0_m       = float(meta.get("r0_m",  INITIAL_RANGE_M))
    v_m_s      = float(meta.get("v_m_s", DEFAULT_CLOSING_VELOCITY_M_S))
    sea_state  = int(meta.get("sea_state", -1))
    seed       = int(meta.get("seed",      -1))

    records = build_tx_records(
        requests           = requests,
        events             = events,
        received_seqs      = received,
        simulator_spawn_ns = spawn_ns,
        r0_m               = r0_m,
        v_m_s              = v_m_s,
    )
    return CellAnalysis(cell_dir=cell_dir, sea_state=sea_state, seed=seed,
                        records=records, summary=summary)


def analyse(out_dir: Path) -> list[CellAnalysis]:
    """Walk ``out_dir`` for every cell with a cell_meta.json + tx_log.csv."""
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

def write_per_vs_range_csv(analyses:    Sequence[CellAnalysis],
                           path:        Path,
                           bin_edges_m: Sequence[float] = DEFAULT_BIN_EDGES_M) -> None:
    """Long-form per-cell per-bin counts. One row per (cell, bin)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["sea_state", "seed", "range_bin_lo_m", "range_bin_hi_m",
                    "n_total", "n_received", "per"])
        for a in analyses:
            bins = aggregate_per(a.records, bin_edges_m)
            for b in bins:
                w.writerow([
                    a.sea_state,
                    a.seed,
                    f"{b.lo_m:.1f}",
                    f"{b.hi_m:.1f}",
                    b.n_total,
                    b.n_received,
                    ("" if math.isnan(b.per) else f"{b.per:.6f}"),
                ])


def write_pooled_per_csv(analyses:    Sequence[CellAnalysis],
                         path:        Path,
                         bin_edges_m: Sequence[float] = DEFAULT_BIN_EDGES_M) -> None:
    """Pooled-across-seeds per-(sea_state, bin) PER + Wilson 95% CI.

    This is the table that backs the paper figure's CI bands — one row
    per (sea_state, range_bin), with the count pooled across seeds so the
    Wilson interval reflects the full sample size rather than per-seed
    sub-samples.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    by_state: dict[int, list[CellAnalysis]] = {}
    for a in analyses:
        by_state.setdefault(a.sea_state, []).append(a)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["sea_state", "range_bin_center_m",
                    "n_total", "per", "per_ci_lo", "per_ci_hi"])
        for ss in sorted(by_state):
            for b in _aggregate_across_seeds(by_state[ss], ss, bin_edges_m):
                if math.isnan(b.per):
                    w.writerow([ss, f"{b.center_m:.1f}", b.n_total, "", "", ""])
                else:
                    w.writerow([
                        ss,
                        f"{b.center_m:.1f}",
                        b.n_total,
                        f"{b.per:.6f}",
                        f"{b.ci_lo:.6f}",
                        f"{b.ci_hi:.6f}",
                    ])


def write_processing_table_csv(analyses: Sequence[CellAnalysis],
                               path:     Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["sea_state", "seed", "mean_us", "p99_us",
                    "max_us", "underruns"])
        for a in analyses:
            pt = (a.summary or {}).get("processing_time", {}) or {}
            w.writerow([
                a.sea_state, a.seed,
                pt.get("mean_us",       ""),
                pt.get("p99_us",        ""),
                pt.get("max_us",        ""),
                pt.get("underrun_count", ""),
            ])


# ---------------------------------------------------------------------------
# Figure
# ---------------------------------------------------------------------------

@dataclass
class PooledBin:
    center_m: float
    per:      float           # NaN when n_total == 0
    ci_lo:    float           # NaN when n_total == 0
    ci_hi:    float
    n_total:  int


def _aggregate_across_seeds(analyses:    Sequence[CellAnalysis],
                            sea_state:   int,
                            bin_edges_m: Sequence[float]) -> list[PooledBin]:
    """For one sea state, pool seeds bin-by-bin into per-bin PER + CI.

    Pooled counts (n_received / n_total across all seeds) are the correct
    estimator for an unequal-n binomial mean — better than averaging
    per-seed PERs. The Wilson CI is computed on the pooled total.
    """
    n_bins = len(bin_edges_m) - 1
    tot = [0] * n_bins
    rcv = [0] * n_bins
    for a in analyses:
        if a.sea_state != sea_state:
            continue
        for i, b in enumerate(aggregate_per(a.records, bin_edges_m)):
            tot[i] += b.n_total
            rcv[i] += b.n_received
    out: list[PooledBin] = []
    for i in range(n_bins):
        center  = 0.5 * (bin_edges_m[i] + bin_edges_m[i + 1])
        if tot[i] == 0:
            out.append(PooledBin(center, float("nan"),
                                 float("nan"), float("nan"), 0))
            continue
        n_failed = tot[i] - rcv[i]
        per      = n_failed / tot[i]
        lo, hi   = wilson_ci(n_failed, tot[i])
        out.append(PooledBin(center, per, lo, hi, tot[i]))
    return out


def _surface_path_threshold_range_m(symbol_period_s: float) -> float | None:
    """Range at which surface-bounce excess delay = one JANUS symbol period.

    Geometry: direct  = sqrt(R² + (z_r-z_s)²)
              surface = sqrt(R² + (z_s+z_r)²)
    Excess Δ = surface − direct.  Solve Δ/c = T_sym for R.
    Returns ``None`` if the threshold falls outside the swept range.
    """
    return _path_threshold_range_m(
        h_direct_m  = abs(RECEIVER_DEPTH_M - SOURCE_DEPTH_M),
        h_bounce_m  = SOURCE_DEPTH_M + RECEIVER_DEPTH_M,
        excess_m    = symbol_period_s * SOUND_SPEED_M_S,
    )


def _bottom_path_threshold_range_m(symbol_period_s: float) -> float | None:
    """Same as surface but for the bottom bounce.

    Geometry: bottom path height = (D - z_s) + (D - z_r) = 90 m for the
    default scene. The bottom-bounce excess delay shrinks slower than
    surface as R grows, so the crossover sits further out than the
    surface one.
    """
    return _path_threshold_range_m(
        h_direct_m  = abs(RECEIVER_DEPTH_M - SOURCE_DEPTH_M),
        h_bounce_m  = (WATER_DEPTH_M - SOURCE_DEPTH_M) + (WATER_DEPTH_M - RECEIVER_DEPTH_M),
        excess_m    = symbol_period_s * SOUND_SPEED_M_S,
    )


def _path_threshold_range_m(h_direct_m: float,
                            h_bounce_m: float,
                            excess_m:   float) -> float | None:
    """Solve √(R² + h_b²) − √(R² + h_d²) = excess for R≥0."""
    # Algebra: let u = R² + h_d². Then √(u + (h_b² − h_d²)) = √u + excess
    # ⇒ u + (h_b² − h_d²) = u + 2·excess·√u + excess²
    # ⇒ √u = (h_b² − h_d² − excess²) / (2·excess)
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


def render_per_figure(analyses:    Sequence[CellAnalysis],
                      savepath:    Path,
                      bin_edges_m: Sequence[float] = DEFAULT_BIN_EDGES_M,
                      sea_states:  Sequence[int]   = DEFAULT_SEA_STATES) -> None:
    """Family of curves (one per sea state) — paper Fig. 4."""
    import matplotlib.pyplot as plt           # local import; plotting is opt
    fig, ax = plt.subplots(figsize=(7.0, 4.2))

    # Vertical annotations at the ranges where each path's excess delay
    # equals one JANUS symbol period (the "narrative payoff" of the figure).
    for label, fn in (("surface (1 sym)", _surface_path_threshold_range_m),
                      ("bottom  (1 sym)", _bottom_path_threshold_range_m)):
        R = fn(JANUS_SYMBOL_PERIOD_S)
        if R is not None and bin_edges_m[0] <= R <= bin_edges_m[-1]:
            ax.axvline(R, linestyle="--", linewidth=0.8, color="0.5")
            ax.text(R, 1.02, label, rotation=90, va="bottom", ha="right",
                    fontsize=8, color="0.4")

    has_data = False
    for ss in sea_states:
        agg = _aggregate_across_seeds(analyses, ss, bin_edges_m)
        valid = [b for b in agg if not math.isnan(b.per)]
        if not valid:
            continue
        has_data = True
        xs  = [b.center_m for b in valid]
        ys  = [b.per      for b in valid]
        los = [b.ci_lo    for b in valid]
        his = [b.ci_hi    for b in valid]
        line, = ax.plot(xs, ys, marker="o", label=f"sea state {ss}")
        ax.fill_between(xs, los, his,
                        alpha=0.18, color=line.get_color(),
                        linewidth=0)
    ax.set_xlabel("Range at TX (m)")
    ax.set_ylabel("Packet error rate")
    ax.set_ylim(-0.02, 1.05)
    ax.set_xlim(bin_edges_m[0], bin_edges_m[-1])
    ax.grid(True, alpha=0.3)
    if has_data:
        ax.legend(loc="best", frameon=False)
    savepath.parent.mkdir(parents=True, exist_ok=True)
    plotting.save_figure(fig, savepath)


# ---------------------------------------------------------------------------
# CDC driver — per-cell pre/post hooks
# ---------------------------------------------------------------------------

class _JanusDriver:
    """Owns the CDC consoles and the TX-loop thread for one cell at a time."""

    def __init__(self,
                 modem_a_serial:        str,
                 modem_b_serial:        str,
                 *,
                 sea_state:             int,
                 seed:                  int,
                 cadence_s:             float,
                 first_request_delay_s: float,
                 max_packets:           int,
                 range_floor_m:         float,
                 r0_m:                  float,
                 v_m_s:                 float,
                 rx_grace_s:            float,
                 cdc_factory:           Callable[..., CdcConsole] = CdcConsole.attach,
                 ready_timeout_s:       float = 15.0) -> None:
        self.modem_a_serial         = modem_a_serial
        self.modem_b_serial         = modem_b_serial
        self.sea_state              = int(sea_state)
        self.seed                   = int(seed)
        self.cadence_s              = float(cadence_s)
        self.first_request_delay_s  = float(first_request_delay_s)
        self.max_packets            = int(max_packets)
        self.range_floor_m          = float(range_floor_m)
        self.r0_m                   = float(r0_m)
        self.v_m_s                  = float(v_m_s)
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
        self._spawns[handle.cell_id] = spawn_ns
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

        done = threading.Event(); stop = threading.Event()
        self._done_evts[handle.cell_id] = done
        self._stop_evts[handle.cell_id] = stop
        worker = threading.Thread(
            target = self._tx_loop,
            args   = (handle.cell_id, cdc_a, cdc_b, done, stop, spawn_ns),
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

        # Flush tx_log.csv and cell_meta.json before detaching CDCs (so
        # late-arriving RX prints still land in modem_b_cdc.log).
        tx_log = self._tx_logs.pop(handle.cell_id, [])
        self._write_tx_log(handle.cell_dir, tx_log)
        self._write_cell_meta(handle.cell_dir,
                              spawn_ns=self._spawns.pop(handle.cell_id, 0))

        # Give modem B a beat to print any in-flight decode block.
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
                 stop:     threading.Event,
                 spawn_ns: int) -> None:
        tx_log = self._tx_logs[cell_id]
        if self.first_request_delay_s > 0.0:
            stop.wait(self.first_request_delay_s)
        seq = 0
        while seq < self.max_packets and not stop.is_set():
            # Bail if either CDC reader thread died mid-cell. A dead modem-B
            # reader means RX prints don't land in the log → packets look
            # lost when they actually decoded. Continuing under that
            # condition produces an inverted PER-vs-range curve (early
            # packets captured, late ones invisible) and burns wall-clock
            # on data we can't use. Better to abort the cell and let the
            # sweep advance.
            if not cdc_a.is_reader_alive() or not cdc_b.is_reader_alive():
                which = "A" if not cdc_a.is_reader_alive() else "B"
                sys.stderr.write(f"[exp3] cell={cell_id[:8]} seq={seq}: "
                                 f"modem-{which} CDC reader died — aborting cell "
                                 "to avoid corrupted PER measurements.\n")
                break
            now_ns  = time.monotonic_ns()
            t_s     = (now_ns - spawn_ns) / 1e9
            rng_m   = self.r0_m + self.v_m_s * t_s
            if rng_m <= self.range_floor_m:
                break
            payload = f"PROBE {seq:03d}"
            try:
                cdc_a.send_janus_011_01_tx(payload)
            except Exception as exc:                            # noqa: BLE001
                sys.stderr.write(f"[exp3] cell={cell_id[:8]} seq={seq} "
                                 f"TX failed: {exc}\n")
                # If the TX-side TTY just went away, the next iteration's
                # is_reader_alive() will catch it and bail; no further
                # special-case handling needed here.
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
            r0_m               = self.r0_m,
            v_m_s              = self.v_m_s,
            range_floor_m      = self.range_floor_m,
            cadence_s          = self.cadence_s,
            max_packets        = self.max_packets,
            sea_state          = self.sea_state,
            seed               = self.seed,
        )
        (cell_dir / "cell_meta.json").write_text(json.dumps(meta, indent=2))


# ---------------------------------------------------------------------------
# Sweep wiring
# ---------------------------------------------------------------------------

def _max_packets_for_run(r0_m: float,
                         v_m_s: float,
                         floor_m: float,
                         cadence_s: float) -> int:
    """Upper bound on TXs before range hits the floor. Includes a small
    safety margin so the TX loop's drift doesn't truncate the trailing bin."""
    if v_m_s >= 0:
        return 10_000        # opening or stationary — let stop_condition rule
    travel_s = (r0_m - floor_m) / abs(v_m_s)
    return int(math.ceil(travel_s / cadence_s)) + 2


def _per_cell_sweep(*,
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
                    max_packets_override:  int | None) -> None:
    """Run one (sea_state, seed) cell as a one-cell Sweep."""
    if max_packets_override is not None and max_packets_override > 0:
        max_packets = int(max_packets_override)
    else:
        max_packets = _max_packets_for_run(
            r0_m      = INITIAL_RANGE_M,
            v_m_s     = velocity_m_s,
            floor_m   = RANGE_FLOOR_M,
            cadence_s = cadence_s,
        )
    driver = _JanusDriver(
        modem_a_serial         = modem_a_serial,
        modem_b_serial         = modem_b_serial,
        sea_state              = sea_state,
        seed                   = seed,
        cadence_s              = cadence_s,
        first_request_delay_s  = first_request_delay_s,
        max_packets            = max_packets,
        range_floor_m          = RANGE_FLOOR_M,
        r0_m                   = INITIAL_RANGE_M,
        v_m_s                  = velocity_m_s,
        rx_grace_s             = rx_grace_s,
    )
    sweep = Sweep(
        template_path     = TEMPLATE,
        parameters        = {"sea_state": [sea_state], "seed": [seed]},
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


def _print_summary(analyses:    Sequence[CellAnalysis],
                   bin_edges_m: Sequence[float]) -> None:
    by_state: dict[int, list[CellAnalysis]] = {}
    for a in analyses:
        by_state.setdefault(a.sea_state, []).append(a)
    sys.stderr.write("[exp3] PER vs range (pooled across seeds, Wilson 95% CI):\n")
    for ss in sorted(by_state):
        sys.stderr.write(f"  sea state {ss} ({len(by_state[ss])} cell(s)):\n")
        for b in _aggregate_across_seeds(by_state[ss], ss, bin_edges_m):
            if math.isnan(b.per):
                sys.stderr.write(f"     R={b.center_m:7.1f} m   "
                                 f"PER=  nan                 n={b.n_total:4d}\n")
            else:
                sys.stderr.write(
                    f"     R={b.center_m:7.1f} m   "
                    f"PER={b.per:5.3f} "
                    f"[{b.ci_lo:5.3f}, {b.ci_hi:5.3f}]   "
                    f"n={b.n_total:4d}\n")


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
    p.add_argument("--sea-states", default=",".join(str(s)
                                                    for s in DEFAULT_SEA_STATES),
                   help="comma-separated Wenz sea state indices "
                        f"(default {','.join(str(s) for s in DEFAULT_SEA_STATES)})")
    p.add_argument("--seeds", type=int, default=DEFAULT_SEEDS,
                   help=f"number of seeds per sea state (default {DEFAULT_SEEDS}). "
                        "Each cell runs the simulator once for ~400 s of "
                        f"approach plus ~{DEFAULT_RX_GRACE_S:.0f} s RX grace.")
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
    p.add_argument("--velocity-m-s", type=float,
                   default=DEFAULT_CLOSING_VELOCITY_M_S,
                   help="closing velocity of modem A in m/s (negative = "
                        f"approaching, default {DEFAULT_CLOSING_VELOCITY_M_S}). "
                        "Drop further (e.g. -0.5) if cargo decode is still "
                        "fragile — slower closing keeps the symbol-window "
                        "drift over a cargo within the firmware demod's "
                        "fixed-grid tolerance.")
    p.add_argument("--modem-a-serial", default="OA-2-1")
    p.add_argument("--modem-b-serial", default="OA-2-2")
    p.add_argument("--retries", type=int, default=0)
    p.add_argument("--log-raw-rx", action="store_true",
                   help="enable raw-RX WAV logging (off by default; PER "
                        "analysis doesn't need it and WAVs balloon disk usage)")
    p.add_argument("--max-packets", type=int, default=None,
                   help="cap TX packets per cell (overrides the R(t)-derived "
                        "default). Use for quick-check runs — e.g. "
                        "`--sea-states 5 --seeds 1 --max-packets 5` runs one "
                        "cell in ≈45 s so you can eyeball the CDC log and "
                        "SNR before committing to the full sweep.")
    p.add_argument("--analysis-only", action="store_true",
                   help="don't run the sweep — only post-process artifacts "
                        "already present in --out")
    args = p.parse_args(argv)

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    sea_states = [int(s) for s in args.sea_states.split(",") if s.strip()]
    seeds      = list(range(int(args.seeds)))

    if not args.analysis_only:
        for ss in sea_states:
            for seed in seeds:
                _per_cell_sweep(
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
                    max_packets_override  = args.max_packets,
                )

    analyses = analyse(out_dir)
    if not analyses:
        sys.stderr.write("[exp3] no analysable cells found in "
                         f"{out_dir} — aborting\n")
        return 3

    write_per_vs_range_csv(analyses, out_dir / "per_vs_range.csv")
    write_pooled_per_csv  (analyses, out_dir / "per_vs_range_pooled.csv")
    write_processing_table_csv(analyses, out_dir / "processing_table.csv")
    render_per_figure(analyses, out_dir / "fig_janus_per.pdf",
                      sea_states=sea_states)
    _print_summary(analyses, DEFAULT_BIN_EDGES_M)
    return 0


if __name__ == "__main__":
    sys.exit(main())
