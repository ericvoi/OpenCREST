"""Sweep orchestrator.

The :class:`Sweep` runs the openCREST binary once per cell in a parameter
cross-product. For each cell it:

1. Renders the scenario YAML from a Jinja2 template into
   ``<out_dir>/<cell_id>/scenario.yaml``.
2. Spawns ``<binary> <scenario.yaml>`` as a subprocess; tees stdout/stderr
   to ``<cell>/stdout.log`` and ``<cell>/stderr.log``.
3. Waits ``duration_s`` (or until a per-cell ``stop_condition`` callback
   returns true), then sends SIGTERM and waits for graceful exit.
4. Verifies ``<cell>/<scenario_name>_summary.json`` exists. Records
   pass/fail.
5. After all cells finish, writes ``<out_dir>/sweep_index.csv`` mapping
   ``cell_id -> params + summary path + exit code``.

The orchestrator is intentionally agnostic to what the experiment is doing
*inside* a cell. Experiment-specific behaviour (issuing CDC commands,
deciding stop conditions, post-processing) is supplied via callbacks.

The openCREST binary itself is opaque -- nothing here is openCREST-specific
beyond the artifact naming convention defined in Session D.

Multi-hour-sweep hardening
--------------------------
Two features matter when a single ``Sweep.run()`` is expected to last hours:

* ``progress=True`` (default) prints a one-line transition for every cell
  start and cell completion to ``progress_stream`` (stderr by default), so
  a sweep that's been running silently for 3 hours is still legible.
* ``max_cell_runtime_s`` is an optional hard kill timer that fires
  independently of ``duration_s`` / ``stop_condition``. It is the safety
  net for message-count-driven experiments that set ``duration_s`` to
  effectively infinity -- a stuck modem otherwise hangs the cell until
  ``duration_s`` elapses, which may be the entire planned sweep budget.

The message-count idiom looks like::

    def stop_when_1000_messages(handle):
        events = handle.cell_dir / "modem_a_events.jsonl"
        if not events.is_file():
            return False
        with events.open() as fp:
            return sum(1 for _ in fp) >= 1000

    Sweep(
        ...,
        duration_s         = float("inf"),    # no soft time limit
        stop_condition     = stop_when_1000_messages,
        max_cell_runtime_s = 4 * 3600.0,      # bail after 4 h if stuck
    ).run()
"""
from __future__ import annotations

import csv
import itertools
import math
import os
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, IO, Iterable, Mapping, Sequence

from . import scenario_template as st


CellId = str
Params = dict[str, Any]
StopCondition = Callable[["CellHandle"], bool]
CellHook = Callable[["CellHandle"], None]


@dataclass
class CellHandle:
    """Per-cell view passed to user hooks (CDC drivers, stop checks)."""
    cell_id: CellId
    params: Params
    cell_dir: Path
    scenario_path: Path
    process: subprocess.Popen | None = None
    started_at: float = 0.0      # time.monotonic() at spawn
    summary_path: Path | None = None


@dataclass
class CellResult:
    cell_id: CellId
    params: Params
    cell_dir: Path
    exit_code: int | None
    summary_path: Path | None
    error: str = ""

    @property
    def ok(self) -> bool:
        return self.exit_code == 0 and self.summary_path is not None \
            and self.summary_path.is_file() and not self.error


@dataclass
class Sweep:
    """Drive the openCREST binary across a parameter cross-product.

    Parameters
    ----------
    template_path
        Jinja2 scenario template (``.yaml.j2``).
    parameters
        Mapping of parameter name -> list of values. The cross-product of
        these lists is the cell set. A scalar value (not a list) is treated
        as a single-element list.
    binary
        Path to the openCREST executable.
    out_dir
        Root directory for per-cell artifacts. One subdirectory per cell.
    duration_s
        Soft time limit: SIGTERM is sent after this many seconds and the
        cell is expected to exit cleanly (success). ``stop_condition``
        may terminate sooner. Set to ``float("inf")`` for purely
        message-count-driven cells (pair with ``max_cell_runtime_s``).
    parallel
        Number of cells to run concurrently. Default 1 because two-modem
        scenarios contend for the same USB devices; experiments with mock
        binaries may raise this.
    extra_params
        Constant parameters that apply to every cell (e.g. modem serials,
        ``channel_gain_db``). Merged into each cell's params before rendering.
    pre_run, post_run
        Optional hooks invoked just after process spawn / just after process
        exit. Use ``pre_run`` to attach a CDC console; ``post_run`` for any
        per-cell teardown.
    stop_condition
        Optional callback polled once per ``poll_interval_s``. Returning
        ``True`` makes the runner SIGTERM the cell early (counts as success).
    max_cell_runtime_s
        Optional hard kill timer. If set, a cell that exceeds this wall-clock
        budget is forcefully terminated and marked **failed** with a watchdog
        reason in ``CellResult.error``. Distinct from ``duration_s``: the
        soft limit is the experiment's planned runtime; the watchdog is a
        safety net for the message-count-driven mode where the soft limit is
        infinity. ``None`` (default) disables the watchdog.
    retries
        Number of extra attempts to make if a cell fails (default 0). Each
        retry overwrites the cell's stdout/stderr/scenario.yaml in place;
        the final attempt's logs are what survive. Useful when the failure
        mode is a transient USB-side hiccup unrelated to the YAML.
    progress, progress_stream
        When ``progress`` is true (default), every cell start and completion
        emits a single line to ``progress_stream`` (``sys.stderr`` by
        default) so long sweeps are legible. Set ``progress=False`` to mute.
    """
    template_path: str | Path
    parameters: Mapping[str, Any]
    binary: str | Path
    out_dir: str | Path
    duration_s: float
    parallel: int = 1
    extra_params: Mapping[str, Any] = field(default_factory=dict)
    pre_run: CellHook | None = None
    post_run: CellHook | None = None
    stop_condition: StopCondition | None = None
    sigterm_grace_s: float = 10.0
    poll_interval_s: float = 0.5
    max_cell_runtime_s: float | None = None
    retries: int = 0
    progress: bool = True
    progress_stream: IO[str] = field(default_factory=lambda: sys.stderr)

    # Populated by run()
    results: list[CellResult] = field(default_factory=list, init=False)

    # Progress state (rebuilt at run() entry).
    _progress_lock: threading.Lock = field(default_factory=threading.Lock,
                                           init=False, repr=False)
    _progress_done: int = field(default=0, init=False, repr=False)
    _progress_ok: int   = field(default=0, init=False, repr=False)
    _progress_fail: int = field(default=0, init=False, repr=False)
    _progress_total: int = field(default=0, init=False, repr=False)
    _progress_started_at: float = field(default=0.0, init=False, repr=False)

    # --- public API ------------------------------------------------------

    def cells(self) -> list[Params]:
        """Materialize the parameter cross-product into a list of dicts."""
        normalised: dict[str, Sequence[Any]] = {}
        for key, val in self.parameters.items():
            if isinstance(val, (list, tuple)):
                normalised[key] = list(val)
            else:
                normalised[key] = [val]

        keys = list(normalised.keys())
        out: list[Params] = []
        for combo in itertools.product(*[normalised[k] for k in keys]):
            cell = dict(self.extra_params)
            cell.update(zip(keys, combo))
            out.append(cell)
        return out

    def run(self) -> list[CellResult]:
        """Execute every cell. Returns the per-cell result list. Idempotent
        in the sense that calling ``run()`` again would re-execute every
        cell from scratch (and overwrite outputs)."""
        out_root = Path(self.out_dir)
        out_root.mkdir(parents=True, exist_ok=True)

        cells = self.cells()
        results: list[CellResult] = []

        self._progress_done   = 0
        self._progress_ok     = 0
        self._progress_fail   = 0
        self._progress_total  = len(cells)
        self._progress_started_at = time.monotonic()

        if self._progress_total > 0 and self.duration_s != float("inf") \
                and self.max_cell_runtime_s is None:
            # Soft upper bound on total wall-clock budget; useful sanity figure
            # when the sweep takes hours. With infinite duration_s and no
            # watchdog there's no meaningful estimate.
            budget_per_cell = self.duration_s
        elif self.max_cell_runtime_s is not None:
            budget_per_cell = self.max_cell_runtime_s
        else:
            budget_per_cell = None
        self._emit_sweep_start(self._progress_total, budget_per_cell)

        if self.parallel <= 1:
            for params in cells:
                results.append(self._run_cell(params, out_root))
        else:
            with ThreadPoolExecutor(max_workers=self.parallel) as pool:
                futures = [pool.submit(self._run_cell, p, out_root) for p in cells]
                for fut in as_completed(futures):
                    results.append(fut.result())

        self._emit_sweep_done()

        # Stable ordering by params for the index CSV
        results.sort(key=lambda r: r.cell_id)
        self.results = results
        self._write_sweep_index(out_root, results)
        return results

    def collect_results(self):
        """Walk ``out_dir``/* and return a long-form pandas DataFrame keyed
        by ``(cell_id, ...)`` -- one row per processing-time snapshot plus
        one row per message event.

        Implemented thinly in terms of :mod:`metrics_loader`; defined here
        so callers can use the spec's ``sweep.collect_results()`` shape
        without reaching into the loader module.
        """
        from . import metrics_loader     # local import: pandas is heavy
        return metrics_loader.load_sweep(Path(self.out_dir))

    # --- internals -------------------------------------------------------

    def _run_cell(self, params: Params, out_root: Path) -> CellResult:
        cell_id = st.cell_id(params)
        cell_dir = out_root / cell_id
        cell_dir.mkdir(parents=True, exist_ok=True)

        cell_start_wall = time.monotonic()
        self._emit_cell_start(cell_id, params)
        result: CellResult | None = None
        try:
            total_attempts = max(1, 1 + int(self.retries))
            for attempt in range(total_attempts):
                if attempt > 0:
                    self._emit_cell_retry(cell_id, attempt + 1, total_attempts,
                                          result.error if result else "no result")
                result = self._run_cell_inner(params, cell_id, cell_dir)
                if result.ok:
                    break
            return result
        finally:
            self._emit_cell_done(cell_id, result,
                                 time.monotonic() - cell_start_wall)

    def _run_cell_inner(self, params: Params, cell_id: CellId,
                        cell_dir: Path) -> CellResult:

        # The template controls output_directory. We override so artifacts
        # land inside the per-cell folder regardless of what the template
        # default says, and the experiment driver doesn't have to thread
        # the directory through itself.
        render_params = dict(params)
        render_params["output_directory"] = str(cell_dir.resolve())

        scenario_path = cell_dir / "scenario.yaml"
        try:
            st.render_to_file(self.template_path, render_params, scenario_path)
        except Exception as exc:                                # render failure
            return CellResult(cell_id, params, cell_dir, None, None,
                              error=f"render failed: {exc}")

        # Pre-render the scenario name from the YAML so we can predict the
        # summary path.
        try:
            scenario_obj = st.validate_yaml(scenario_path.read_text())
            scenario_name = str(scenario_obj.get("name", "run"))
        except Exception as exc:
            return CellResult(cell_id, params, cell_dir, None, None,
                              error=f"scenario parse failed: {exc}")

        # The simulator writes <output_dir>/<name>_summary.json by default
        # (see src/simulator/run_summary.cpp::resolve_run_summary_path).
        expected_summary = cell_dir / f"{scenario_name}_summary.json"

        handle = CellHandle(
            cell_id        = cell_id,
            params         = params,
            cell_dir       = cell_dir,
            scenario_path  = scenario_path,
            summary_path   = expected_summary,
        )

        stdout_log = open(cell_dir / "stdout.log", "w", buffering=1)
        stderr_log = open(cell_dir / "stderr.log", "w", buffering=1)

        try:
            proc = subprocess.Popen(
                [str(self.binary), str(scenario_path)],
                stdout=stdout_log,
                stderr=stderr_log,
                # New process group so we can SIGTERM the whole tree if the
                # binary spawned children.
                preexec_fn=os.setsid if os.name == "posix" else None,
            )
        except FileNotFoundError as exc:
            stdout_log.close()
            stderr_log.close()
            return CellResult(cell_id, params, cell_dir, None, None,
                              error=f"binary not found: {exc}")

        handle.process = proc
        handle.started_at = time.monotonic()

        if self.pre_run is not None:
            try:
                self.pre_run(handle)
            except Exception as exc:                            # noqa: BLE001
                # Hook failure shouldn't kill the cell -- log to stderr.log
                stderr_log.write(f"[runner] pre_run hook failed: {exc}\n")

        exit_code, watchdog_reason = self._wait_or_terminate(handle, proc)

        if self.post_run is not None:
            try:
                self.post_run(handle)
            except Exception as exc:                            # noqa: BLE001
                stderr_log.write(f"[runner] post_run hook failed: {exc}\n")

        stdout_log.close()
        stderr_log.close()

        summary = expected_summary if expected_summary.is_file() else None
        err = ""
        if watchdog_reason is not None:
            # Watchdog takes precedence over the natural exit-code
            # message: the cell was forcibly killed, exit-code semantics
            # don't really apply.
            err = watchdog_reason
        elif exit_code is None:
            err = "process did not exit after SIGTERM"
        elif exit_code != 0:
            err = f"exit code {exit_code}"
        elif summary is None:
            err = "summary JSON not emitted"

        return CellResult(cell_id, params, cell_dir, exit_code, summary, err)

    def _wait_or_terminate(self, handle: CellHandle,
                           proc: subprocess.Popen
                           ) -> tuple[int | None, str | None]:
        """Poll until one of: the process exits naturally, ``duration_s``
        elapses, ``stop_condition`` returns true, or the watchdog fires.

        Returns ``(exit_code, watchdog_reason)``. ``watchdog_reason`` is
        ``None`` for natural exit / soft-limit / stop-condition paths and
        a short string when ``max_cell_runtime_s`` triggered the kill --
        in which case the cell is treated as failed regardless of the
        exit code (which will typically be ``-SIGTERM`` or ``-SIGKILL``).
        """
        soft_deadline = handle.started_at + self.duration_s
        hard_deadline = (handle.started_at + self.max_cell_runtime_s
                         if self.max_cell_runtime_s is not None
                         else math.inf)

        while True:
            rc = proc.poll()
            if rc is not None:
                return rc, None
            now = time.monotonic()
            if now >= hard_deadline:
                rc = self._terminate(proc)
                return rc, (f"exceeded max_cell_runtime_s "
                            f"({self.max_cell_runtime_s:.1f}s)")
            if now >= soft_deadline:
                break
            if self.stop_condition is not None and self.stop_condition(handle):
                break
            # Sleep until the next deadline or the next poll tick, whichever
            # is sooner. Cap at poll_interval_s so stop_condition is
            # exercised at a predictable cadence.
            next_deadline = min(soft_deadline, hard_deadline)
            sleep_for = min(self.poll_interval_s, max(next_deadline - now, 0.0))
            time.sleep(sleep_for)

        # Soft-limit or stop_condition: graceful SIGTERM, expect clean exit.
        return self._terminate(proc), None

    def _terminate(self, proc: subprocess.Popen) -> int | None:
        try:
            if os.name == "posix":
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
        except ProcessLookupError:
            return proc.poll()

        try:
            return proc.wait(timeout=self.sigterm_grace_s)
        except subprocess.TimeoutExpired:
            try:
                if os.name == "posix":
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                else:
                    proc.kill()
            except ProcessLookupError:
                pass
            try:
                return proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                return None

    # --- progress reporter ----------------------------------------------

    def _emit(self, text: str) -> None:
        if not self.progress:
            return
        with self._progress_lock:
            self.progress_stream.write(text + "\n")
            self.progress_stream.flush()

    @staticmethod
    def _format_hms(seconds: float) -> str:
        if not math.isfinite(seconds) or seconds < 0:
            return "unknown"
        s = int(round(seconds))
        h, rem = divmod(s, 3600)
        m, s = divmod(rem, 60)
        if h:
            return f"{h}h{m:02d}m{s:02d}s"
        if m:
            return f"{m}m{s:02d}s"
        return f"{s}s"

    def _emit_sweep_start(self, total: int,
                          budget_per_cell: float | None) -> None:
        if total == 0:
            self._emit("[sweep] no cells to run")
            return
        if budget_per_cell is None:
            self._emit(f"[sweep] starting {total} cell(s), wall-clock budget "
                       f"unknown (duration_s=inf, no watchdog)")
            return
        # Total estimated time assumes sequential cell execution and that
        # each cell uses its full budget. Real runs are usually faster
        # when stop_condition fires early.
        total_s = total * budget_per_cell / max(self.parallel, 1)
        self._emit(f"[sweep] starting {total} cell(s); upper-bound "
                   f"~{self._format_hms(total_s)} at parallel={self.parallel}")

    def _emit_cell_start(self, cell_id: CellId, params: Params) -> None:
        if not self.progress:
            return
        # Avoid dumping every param key (some sweeps have lots); show
        # only the parameters that the cells differ in. Cheap heuristic:
        # the sweep-axis keys are exactly self.parameters.keys().
        axis_keys = list(self.parameters.keys())
        salient = " ".join(f"{k}={params[k]}" for k in axis_keys
                           if k in params) or "..."
        with self._progress_lock:
            started = self._progress_done + 1
        self._emit(f"[sweep] cell {cell_id} starting "
                   f"(scheduled {started}/{self._progress_total}) {salient}")

    def _emit_cell_retry(self, cell_id: CellId, attempt: int,
                         total_attempts: int, prior_error: str) -> None:
        if not self.progress:
            return
        # Trim long error messages — the watchdog and binary-not-found reasons
        # are useful at a glance; longer dumps just spam the log.
        snippet = prior_error.splitlines()[0] if prior_error else ""
        if len(snippet) > 80:
            snippet = snippet[:77] + "..."
        self._emit(f"[sweep] cell {cell_id} retry {attempt}/{total_attempts} "
                   f"(prior: {snippet})")

    def _emit_cell_done(self, cell_id: CellId,
                        result: CellResult | None,
                        elapsed_s: float) -> None:
        if not self.progress:
            return
        with self._progress_lock:
            self._progress_done += 1
            if result is not None and result.ok:
                self._progress_ok += 1
                state = "ok"
                detail = ""
            else:
                self._progress_fail += 1
                state = "FAIL"
                detail = f" ({result.error})" if result and result.error \
                         else " (no result)"
            done = self._progress_done
        eta = ""
        if 0 < done < self._progress_total:
            elapsed_total = time.monotonic() - self._progress_started_at
            mean_per_cell = elapsed_total / done
            remaining = (self._progress_total - done) * mean_per_cell \
                        / max(self.parallel, 1)
            eta = f"; eta {self._format_hms(remaining)}"
        self._emit(f"[sweep] cell {cell_id} {state} in "
                   f"{self._format_hms(elapsed_s)} "
                   f"[{done}/{self._progress_total} done, "
                   f"{self._progress_ok} ok, "
                   f"{self._progress_fail} fail{eta}]{detail}")

    def _emit_sweep_done(self) -> None:
        if not self.progress or self._progress_total == 0:
            return
        elapsed = time.monotonic() - self._progress_started_at
        self._emit(f"[sweep] complete: {self._progress_ok} ok, "
                   f"{self._progress_fail} fail, "
                   f"total {self._format_hms(elapsed)}")

    def _write_sweep_index(self, out_root: Path,
                           results: Sequence[CellResult]) -> None:
        index_path = out_root / "sweep_index.csv"
        # Param key union, sorted for stability.
        param_keys: list[str] = sorted({k for r in results for k in r.params})
        with index_path.open("w", newline="") as fp:
            w = csv.writer(fp)
            w.writerow(["cell_id", *param_keys,
                        "exit_code", "summary_path", "ok", "error"])
            for r in results:
                w.writerow([
                    r.cell_id,
                    *[r.params.get(k, "") for k in param_keys],
                    "" if r.exit_code is None else r.exit_code,
                    str(r.summary_path) if r.summary_path else "",
                    "1" if r.ok else "0",
                    r.error,
                ])
