"""Experiment 2 — two-way ranging accuracy at R=500 m.

Sweeps four channel configurations:

  a — propagation delay only
  b — 3-tap static multipath + bulk Farrow Doppler + clock offset + noise
  c — 5-tap geometric scene (stationary) + noise
  d — config (c) + 2 tonal interferers

For each config, the driver spawns the simulator once, attaches a CDC
console to both modems, then issues ``--num-requests`` ranging requests
back-to-back on modem A. Cadence is response-driven: send a request,
wait for the ``Range: %.2fm`` line on modem A's CDC (up to
``--response-timeout-s``), wait ``--inter-request-delay-s``, repeat. A
single simulator instance per config keeps the modems calibrated across
the batch.

Outputs (under ``--out``, default ``experiments/results/exp2/``):

  <cfg>/scenario.yaml
  <cfg>/<name>_summary.json
  <cfg>/modem_*_cdc.log
  <cfg>/requests.jsonl           one row per issued request

  ranging_results.csv            per-request error data
  processing_table.csv           per-config processing-time table
  fig_ranging.pdf                violin per config

Success rates and processing-time figures are printed to stderr after
analysis. ``experiments/exp2_verify_snr.py`` is a fast sanity check for
each config's SNR before committing to the full batch.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import re
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np

from experiments.lib import plotting
from experiments.lib.cdc_console import CdcConsole
from experiments.lib.runner import CellHandle, Sweep


REPO = Path(__file__).resolve().parents[1]
CONFIG_DIR = REPO / "experiments" / "configs" / "exp2"

CONFIG_IDS = ("a", "b", "c", "d")
TEMPLATES = {c: CONFIG_DIR / f"exp2_ranging_{c}.yaml.j2" for c in CONFIG_IDS}

CONFIGURED_RANGE_M = 500.0

DEFAULT_NUM_REQUESTS = 200
DEFAULT_INTER_REQUEST_DELAY_S = 1.0
DEFAULT_RESPONSE_TIMEOUT_S = 5.0
# Extra wait between the "Simulation running" banner and the first ranging
# request. The firmware needs a beat to settle into its autonomous TX/RX
# cycling after enter_hil_mode(); requests issued during that window are
# dropped.
DEFAULT_FIRST_REQUEST_DELAY_S = 5.0

# Single measurements with absolute error above this threshold are treated
# as firmware/implementation artifacts (e.g. tonal-induced false sync).
# Raw values are preserved in ranging_results.csv with is_outlier=1 but
# excluded from the violin and inlier statistics. Set well above plausible
# channel residuals at the configured range.
DEFAULT_OUTLIER_THRESHOLD_M = 50.0

# Per-cell wall-clock ceiling: worst case is ``num_requests * (response
# timeout + inter-request delay)``. Cells that complete faster exit via
# the stop_condition.
DEFAULT_MAX_CELL_RUNTIME_S = 1800.0
SIGTERM_GRACE_S = 6.0

RANGE_RE = re.compile(r"Range:\s*([-+]?\d+(?:\.\d+)?)\s*m")


# ---------------------------------------------------------------------------
# CDC range parser
# ---------------------------------------------------------------------------

def parse_range_line(line: str) -> float | None:
    """Return the metres parsed out of a firmware ``Range: %.2fm`` print, or
    ``None`` if ``line`` doesn't look like a range readout. Tolerant of
    surrounding whitespace, menu redraws, and timestamp prefixes.
    """
    if not isinstance(line, str):
        return None
    m = RANGE_RE.search(line)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# Per-cell request record
# ---------------------------------------------------------------------------

@dataclass
class RangingRecord:
    request_id: int
    request_ts_ns: int
    response_ts_ns: int | None       # None on timeout
    response_text: str | None        # raw CDC line, None on timeout
    range_m: float | None            # parsed range, None on timeout

    def to_jsonable(self) -> dict:
        return dict(
            request_id      = self.request_id,
            request_ts_ns   = self.request_ts_ns,
            response_ts_ns  = self.response_ts_ns,
            response_text   = self.response_text,
            range_m         = self.range_m,
        )


# ---------------------------------------------------------------------------
# CDC driver — orchestrates the ranging loop per cell
# ---------------------------------------------------------------------------

class _RangingDriver:
    """Per-cell ``pre_run`` / ``post_run`` glue.

    ``pre_run`` attaches a CDC console to each modem, waits for the
    simulator's ready banner, and spawns a worker thread that issues
    ``num_requests`` ranging requests via modem A's CDC. ``post_run``
    flushes the per-cell ``RangingRecord`` list to ``requests.jsonl`` and
    detaches the consoles. ``cell_done(cell_id)`` plugs into the runner's
    ``stop_condition`` so cells end as soon as the worker finishes.
    """

    READY_PATTERN = re.compile(r"Simulation running")

    def __init__(self,
                 modem_a_serial: str,
                 modem_b_serial: str,
                 *,
                 num_requests: int,
                 response_timeout_s: float,
                 inter_request_delay_s: float,
                 first_request_delay_s: float = DEFAULT_FIRST_REQUEST_DELAY_S,
                 cdc_factory: Callable[..., CdcConsole] = CdcConsole.attach,
                 ready_timeout_s: float = 10.0,
                 ) -> None:
        self.modem_a_serial         = modem_a_serial
        self.modem_b_serial         = modem_b_serial
        self.num_requests           = int(num_requests)
        self.response_timeout_s     = float(response_timeout_s)
        self.inter_request_delay_s  = float(inter_request_delay_s)
        self.first_request_delay_s  = float(first_request_delay_s)
        self._cdc_factory           = cdc_factory
        self._ready_timeout         = ready_timeout_s

        self._consoles: dict[str, list[CdcConsole]] = {}
        self._records:  dict[str, list[RangingRecord]] = {}
        self._done_evts: dict[str, threading.Event] = {}
        self._stop_evts: dict[str, threading.Event] = {}
        self._threads:  dict[str, threading.Thread] = {}

    # --- public API used by Sweep ---------------------------------------

    def pre_run(self, handle: CellHandle) -> None:
        stdout = handle.cell_dir / "stdout.log"
        self._wait_for_ready(stdout)
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
        self._records [handle.cell_id] = []
        done = threading.Event()
        stop = threading.Event()
        self._done_evts[handle.cell_id] = done
        self._stop_evts[handle.cell_id] = stop

        worker = threading.Thread(
            target = self._request_loop,
            args   = (handle.cell_id, cdc_a, done, stop),
            name   = f"exp2-rng-{handle.cell_id[:8]}",
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

        records = self._records.pop(handle.cell_id, [])
        out_path = handle.cell_dir / "requests.jsonl"
        with out_path.open("w") as fp:
            for rec in records:
                fp.write(json.dumps(rec.to_jsonable()) + "\n")

        for console in self._consoles.pop(handle.cell_id, []):
            try:
                console.detach()
            except Exception:
                pass
        self._done_evts.pop(handle.cell_id, None)

    def cell_done(self, handle: CellHandle) -> bool:
        """``stop_condition`` hook: True once all ``num_requests`` have been
        issued and waited on for ``handle.cell_id``."""
        evt = self._done_evts.get(handle.cell_id)
        return bool(evt and evt.is_set())

    # --- internals -------------------------------------------------------

    def _wait_for_ready(self, stdout: Path) -> None:
        deadline = time.monotonic() + self._ready_timeout
        while time.monotonic() < deadline:
            if stdout.is_file():
                try:
                    text = stdout.read_text(errors="replace")
                except OSError:
                    text = ""
                if self.READY_PATTERN.search(text):
                    return
            time.sleep(0.1)
        sys.stderr.write(
            "[exp2] simulator did not print 'Simulation running' within "
            f"{self._ready_timeout:.1f}s — proceeding anyway; first few "
            "requests may be lost to startup.\n")

    def _request_loop(self,
                      cell_id: str,
                      cdc_a: CdcConsole,
                      done: threading.Event,
                      stop: threading.Event) -> None:
        records = self._records[cell_id]
        # Let the modem firmware settle into its autonomous TX/RX cycle
        # before the first request, otherwise request_id=0 is consistently
        # dropped.
        if self.first_request_delay_s > 0.0:
            stop.wait(self.first_request_delay_s)
        for i in range(self.num_requests):
            if stop.is_set():
                break
            request_ts_ns = time.monotonic_ns()
            try:
                cdc_a.send_ranging_request()
            except Exception as exc:                            # noqa: BLE001
                sys.stderr.write(f"[exp2] cell={cell_id[:8]} req#{i} send "
                                 f"failed: {exc}\n")
                records.append(RangingRecord(i, request_ts_ns,
                                             None, None, None))
                # Honour the inter-request delay on send error so we don't
                # busy-loop on a wedged TTY.
                stop.wait(self.inter_request_delay_s)
                continue
            try:
                line = cdc_a.expect_line(r"Range:\s*[-+]?\d+(?:\.\d+)?\s*m",
                                         timeout=self.response_timeout_s)
                response_ts_ns = time.monotonic_ns()
                rng = parse_range_line(line)
                records.append(RangingRecord(i, request_ts_ns,
                                             response_ts_ns, line, rng))
            except TimeoutError:
                records.append(RangingRecord(i, request_ts_ns,
                                             None, None, None))
            stop.wait(self.inter_request_delay_s)
        done.set()


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

@dataclass
class ConfigAnalysis:
    config_id: str
    records: list[RangingRecord]
    summary: dict | None
    cell_dir: Path

    @property
    def errors_m(self) -> np.ndarray:
        """All measured errors (no outlier filtering)."""
        return np.array([r.range_m - CONFIGURED_RANGE_M
                         for r in self.records
                         if r.range_m is not None], dtype=float)

    def errors_inlier(self, threshold_m: float) -> np.ndarray:
        """Errors with |error| <= threshold; backs the violin plot and the
        inlier statistics."""
        return np.array([r.range_m - CONFIGURED_RANGE_M
                         for r in self.records
                         if r.range_m is not None
                         and abs(r.range_m - CONFIGURED_RANGE_M) <= threshold_m],
                        dtype=float)

    def outliers(self, threshold_m: float) -> list[RangingRecord]:
        """Successfully-decoded records whose |error| exceeds threshold."""
        return [r for r in self.records
                if r.range_m is not None
                and abs(r.range_m - CONFIGURED_RANGE_M) > threshold_m]

    @property
    def num_requests(self) -> int:
        return len(self.records)

    @property
    def num_responses(self) -> int:
        return sum(1 for r in self.records if r.range_m is not None)

    @property
    def success_rate(self) -> float:
        return self.num_responses / self.num_requests \
               if self.num_requests else 0.0

    @property
    def response_latencies_ms(self) -> np.ndarray:
        out: list[float] = []
        for r in self.records:
            if r.response_ts_ns is None:
                continue
            out.append((r.response_ts_ns - r.request_ts_ns) / 1e6)
        return np.array(out, dtype=float)


def _load_records(cell_dir: Path) -> list[RangingRecord]:
    path = cell_dir / "requests.jsonl"
    out: list[RangingRecord] = []
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
        out.append(RangingRecord(
            request_id     = int(doc["request_id"]),
            request_ts_ns  = int(doc["request_ts_ns"]),
            response_ts_ns = (None if doc.get("response_ts_ns") is None
                              else int(doc["response_ts_ns"])),
            response_text  = doc.get("response_text"),
            range_m        = (None if doc.get("range_m") is None
                              else float(doc["range_m"])),
        ))
    return out


def _load_summary(cell_dir: Path) -> dict | None:
    candidates = sorted(cell_dir.glob("*_summary.json"))
    if not candidates:
        return None
    try:
        return json.loads(candidates[0].read_text())
    except json.JSONDecodeError:
        return None


def analyse(out_dir: Path) -> list[ConfigAnalysis]:
    """Walk ``out_dir`` and return one ``ConfigAnalysis`` per discoverable
    config cell directory."""
    out: list[ConfigAnalysis] = []
    for config_id in CONFIG_IDS:
        cell_dir = _find_cell_dir(out_dir, config_id)
        if cell_dir is None:
            continue
        records = _load_records(cell_dir)
        summary = _load_summary(cell_dir)
        out.append(ConfigAnalysis(config_id, records, summary, cell_dir))
    return out


def _find_cell_dir(out_dir: Path, config_id: str) -> Path | None:
    """Locate the cell directory for ``config_id`` by matching the
    scenario YAML's ``name:`` field. Cell directory names are param-hash
    based so they can't be derived from the config id directly.
    """
    target = f"exp2_ranging_{config_id}"
    for child in sorted(out_dir.iterdir()):
        if not child.is_dir():
            continue
        scenario = child / "scenario.yaml"
        if not scenario.is_file():
            continue
        try:
            text = scenario.read_text()
        except OSError:
            continue
        if f"name: {target}\n" in text or f"name: {target}\r\n" in text:
            return child
    return None


# ---------------------------------------------------------------------------
# CSV / figure writers
# ---------------------------------------------------------------------------

def write_ranging_results_csv(analyses: list[ConfigAnalysis],
                              path: Path,
                              *,
                              outlier_threshold_m: float
                              = DEFAULT_OUTLIER_THRESHOLD_M) -> None:
    """Write every request as a row. ``is_outlier`` is 1 for any successful
    response whose |error| exceeds the threshold, 0 for inlier responses,
    blank for timeouts (no measurement to classify)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow([
            "config", "request_id", "response_text",
            "range_m", "error_m", "response_latency_ms", "is_outlier",
        ])
        for a in analyses:
            for r in a.records:
                if r.range_m is None:
                    w.writerow([a.config_id, r.request_id, "",
                                "", "", "", ""])
                    continue
                err = r.range_m - CONFIGURED_RANGE_M
                latency_ms = ((r.response_ts_ns - r.request_ts_ns) / 1e6
                              if r.response_ts_ns is not None else "")
                w.writerow([
                    a.config_id, r.request_id,
                    (r.response_text or "").strip(),
                    f"{r.range_m:.3f}",
                    f"{err:.3f}",
                    (f"{latency_ms:.3f}" if isinstance(latency_ms, float)
                     else ""),
                    "1" if abs(err) > outlier_threshold_m else "0",
                ])


def write_processing_table_csv(analyses: list[ConfigAnalysis],
                               path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["config", "mean_us", "p99_us", "max_us", "underruns"])
        for a in analyses:
            pt = (a.summary or {}).get("processing_time", {}) or {}
            w.writerow([
                a.config_id,
                pt.get("mean_us", ""),
                pt.get("p99_us",  ""),
                pt.get("max_us",  ""),
                pt.get("underrun_count", ""),
            ])


def render_violin(analyses: list[ConfigAnalysis],
                  savepath: Path,
                  *,
                  outlier_threshold_m: float
                  = DEFAULT_OUTLIER_THRESHOLD_M) -> None:
    """Render one violin per config. Outliers (|error| > threshold) are
    excluded from the plotted distribution so a single artifact value
    doesn't squash every config's violin; they remain in the CSV and
    stderr summary.
    """
    values_by_cat = {a.config_id: a.errors_inlier(outlier_threshold_m)
                     for a in analyses}
    if not values_by_cat:
        return
    fig = plotting.violin_by_category(
        values_by_cat,
        order   = [a.config_id for a in analyses],
        x_label = "Channel configuration",
        y_label = "Range error (m)",
    )
    # Element sizing (labels/ticks) comes from plotting.apply_paper_style().
    plotting.save_figure(fig, savepath)


# ---------------------------------------------------------------------------
# Sweep driver
# ---------------------------------------------------------------------------

def _run_one_config(*,
                    config_id: str,
                    template_path: Path,
                    out_root: Path,
                    binary: Path | str,
                    modem_a_serial: str,
                    modem_b_serial: str,
                    num_requests: int,
                    response_timeout_s: float,
                    inter_request_delay_s: float,
                    first_request_delay_s: float,
                    max_cell_runtime_s: float,
                    log_raw_rx: bool,
                    retries: int,
                    ) -> None:
    """Run one config as a single-cell Sweep so progress, retries, and the
    watchdog are inherited from the runner."""
    driver = _RangingDriver(
        modem_a_serial          = modem_a_serial,
        modem_b_serial          = modem_b_serial,
        num_requests            = num_requests,
        response_timeout_s      = response_timeout_s,
        inter_request_delay_s   = inter_request_delay_s,
        first_request_delay_s   = first_request_delay_s,
    )
    sweep = Sweep(
        template_path     = template_path,
        # Keep ``config`` in the params dict so cell_id varies across
        # configs even though each Sweep renders a single cell.
        parameters        = {"config": [config_id]},
        extra_params      = dict(
            modem_a_serial = modem_a_serial,
            modem_b_serial = modem_b_serial,
            log_raw_rx     = bool(log_raw_rx),
        ),
        binary             = binary,
        out_dir            = out_root,
        duration_s         = float("inf"),       # response-driven cadence
        parallel           = 1,
        sigterm_grace_s    = SIGTERM_GRACE_S,
        retries            = retries,
        max_cell_runtime_s = max_cell_runtime_s,
        pre_run            = driver.pre_run,
        post_run           = driver.post_run,
        stop_condition     = driver.cell_done,
    )
    sweep.run()


def _print_summary(analyses: list[ConfigAnalysis],
                   *,
                   outlier_threshold_m: float
                   = DEFAULT_OUTLIER_THRESHOLD_M) -> None:
    sys.stderr.write(
        f"[exp2] per-config results (outlier threshold |err| > "
        f"{outlier_threshold_m:.1f} m):\n"
    )

    def _stats(arr: np.ndarray) -> tuple[float, float, float, float, float]:
        if not arr.size:
            return (float("nan"),) * 5
        return (
            float(np.mean(arr)),
            float(np.std(arr)),
            float(np.percentile(np.abs(arr), 50)),
            float(np.percentile(np.abs(arr), 95)),
            float(np.percentile(np.abs(arr), 99)),
        )

    for a in analyses:
        errs_all  = a.errors_m
        errs_in   = a.errors_inlier(outlier_threshold_m)
        outliers  = a.outliers(outlier_threshold_m)
        mean, std, p50, p95, p99 = _stats(errs_in)
        sys.stderr.write(
            f"  [{a.config_id}] requests={a.num_requests:3d}  "
            f"responses={a.num_responses:3d}  "
            f"success={a.success_rate * 100:5.1f}%  "
            f"inliers={errs_in.size:3d}  outliers={len(outliers)}\n"
            f"        inlier stats: bias={mean:+.3f} m  std={std:.3f} m  "
            f"|err| p50={p50:.3f}/p95={p95:.3f}/p99={p99:.3f} m\n"
        )
        if outliers:
            sys.stderr.write("        outlier values (kept in CSV, "
                             "excluded from violin / inlier stats):\n")
            for o in outliers:
                err = o.range_m - CONFIGURED_RANGE_M  # type: ignore[operator]
                sys.stderr.write(
                    f"          request_id={o.request_id:3d}  "
                    f"range={o.range_m:.2f} m  err={err:+.2f} m\n"
                )
            # Also show all-inclusive stats so the no-filter answer doesn't
            # require reloading the CSV.
            mean_all, std_all, *_ = _stats(errs_all)
            sys.stderr.write(
                f"        all-inclusive stats (for disclosure): "
                f"bias={mean_all:+.3f} m  std={std_all:.3f} m\n"
            )

    sys.stderr.write("[exp2] processing-time table:\n")
    for a in analyses:
        pt = (a.summary or {}).get("processing_time", {}) or {}
        sys.stderr.write(
            f"  [{a.config_id}] mean={pt.get('mean_us', 'n/a')} us  "
            f"p99={pt.get('p99_us', 'n/a')} us  "
            f"max={pt.get('max_us', 'n/a')} us  "
            f"underruns={pt.get('underrun_count', 'n/a')}\n"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", default="experiments/results/exp2",
                   help="output directory for sweep artifacts")
    p.add_argument("--binary",
                   default=str(REPO / "build" / "openCREST"),
                   help="path to the openCREST simulator binary")
    p.add_argument("--configs", default=",".join(CONFIG_IDS),
                   help="comma-separated subset of {a,b,c,d} to run "
                        f"(default: all four = {','.join(CONFIG_IDS)})")
    p.add_argument("--num-requests", type=int, default=DEFAULT_NUM_REQUESTS,
                   help=f"requests per config (default {DEFAULT_NUM_REQUESTS})")
    p.add_argument("--inter-request-delay-s", type=float,
                   default=DEFAULT_INTER_REQUEST_DELAY_S,
                   help="seconds to wait between a response (or response "
                        "timeout) and the next request "
                        f"(default {DEFAULT_INTER_REQUEST_DELAY_S})")
    p.add_argument("--response-timeout-s", type=float,
                   default=DEFAULT_RESPONSE_TIMEOUT_S,
                   help=f"max wait for a Range: response per request "
                        f"(default {DEFAULT_RESPONSE_TIMEOUT_S})")
    p.add_argument("--first-request-delay-s", type=float,
                   default=DEFAULT_FIRST_REQUEST_DELAY_S,
                   help="additional wait after the simulator-ready banner "
                        "before the first request, to let the modem firmware "
                        "settle into its autonomous TX/RX cycle "
                        f"(default {DEFAULT_FIRST_REQUEST_DELAY_S})")
    p.add_argument("--max-cell-runtime-s", type=float,
                   default=DEFAULT_MAX_CELL_RUNTIME_S,
                   help="watchdog ceiling per config "
                        f"(default {DEFAULT_MAX_CELL_RUNTIME_S})")
    p.add_argument("--modem-a-serial", default="OA-2-1")
    p.add_argument("--modem-b-serial", default="OA-2-2")
    p.add_argument("--retries", type=int, default=0)
    p.add_argument("--log-raw-rx", action="store_true",
                   help="enable raw-RX WAV logging in every config (off by "
                        "default; exp2 analysis only needs the CDC range "
                        "readouts and raw WAVs balloon disk usage)")
    p.add_argument("--outlier-threshold-m", type=float,
                   default=DEFAULT_OUTLIER_THRESHOLD_M,
                   help="absolute-error threshold (metres) above which a "
                        "measurement is flagged is_outlier=1 in the CSV and "
                        "excluded from the violin + inlier stats. Outliers "
                        "are preserved in the CSV and listed in the stderr "
                        f"summary (default {DEFAULT_OUTLIER_THRESHOLD_M})")
    p.add_argument("--analysis-only", action="store_true",
                   help="don't run the sweep — only post-process artifacts "
                        "already present in --out")
    args = p.parse_args(argv)

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    config_ids = [c.strip() for c in args.configs.split(",") if c.strip()]
    bad = [c for c in config_ids if c not in CONFIG_IDS]
    if bad:
        sys.stderr.write(f"[exp2] unknown config(s): {bad}\n")
        return 2

    if not args.analysis_only:
        for cid in config_ids:
            _run_one_config(
                config_id            = cid,
                template_path        = TEMPLATES[cid],
                out_root             = out_dir,
                binary               = args.binary,
                modem_a_serial       = args.modem_a_serial,
                modem_b_serial       = args.modem_b_serial,
                num_requests         = args.num_requests,
                response_timeout_s   = args.response_timeout_s,
                inter_request_delay_s= args.inter_request_delay_s,
                first_request_delay_s= args.first_request_delay_s,
                max_cell_runtime_s   = args.max_cell_runtime_s,
                log_raw_rx           = args.log_raw_rx,
                retries              = args.retries,
            )

    analyses = analyse(out_dir)
    if not analyses:
        sys.stderr.write("[exp2] no analysable cells found in "
                         f"{out_dir} — aborting\n")
        return 3

    write_ranging_results_csv(analyses, out_dir / "ranging_results.csv",
                              outlier_threshold_m=args.outlier_threshold_m)
    write_processing_table_csv(analyses, out_dir / "processing_table.csv")
    render_violin(analyses, out_dir / "fig_ranging.pdf",
                  outlier_threshold_m=args.outlier_threshold_m)
    _print_summary(analyses,
                   outlier_threshold_m=args.outlier_threshold_m)
    return 0


if __name__ == "__main__":
    sys.exit(main())
