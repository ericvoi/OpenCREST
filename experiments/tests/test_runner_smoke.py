"""Smoke test for the Sweep runner.

Exercises the orchestrator end-to-end against a stub binary so the test
runs without USB hardware. Covers:

 * Single-cell sweep renders the YAML, spawns the subprocess, sees the
   summary JSON, and reports ok=True.
 * Multi-cell parameter cross-product writes a stable sweep_index.csv.
 * SIGTERM path exits cleanly within the grace window.
 * Missing-binary path surfaces the error without raising.
 * ``max_cell_runtime_s`` watchdog fires and marks the cell failed when
   the stub ignores SIGTERM.
 * Progress logging emits one start/done line per cell on the requested
   stream and stays silent when disabled.
"""
from __future__ import annotations

import csv
import io
import math
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

from experiments.lib.runner import Sweep


REPO = Path(__file__).resolve().parents[2]
TEMPLATE = REPO / "experiments" / "configs" / "common" / "two_modem_base.yaml.j2"
STUB     = REPO / "experiments" / "tests" / "_stub_binary.py"


def _stub_invocation() -> list[str]:
    """Treat the stub script as the binary by wrapping it in a tiny shell
    launcher so the runner invokes a single executable."""
    return [sys.executable, str(STUB)]


@pytest.fixture
def stub_binary(tmp_path: Path) -> Path:
    """Materialise the stub as an on-disk executable for ``Sweep``."""
    launcher = tmp_path / "openCREST_stub"
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        f"exec {sys.executable} {STUB} \"$@\"\n",
    )
    launcher.chmod(0o755)
    return launcher


@pytest.fixture
def stubborn_binary(tmp_path: Path) -> Path:
    """Stub variant that ignores SIGTERM; exercises the
    ``max_cell_runtime_s`` watchdog -> SIGKILL fallback path."""
    launcher = tmp_path / "openCREST_stubborn"
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        f"exec {sys.executable} {STUB} --ignore-sigterm \"$@\"\n",
    )
    launcher.chmod(0o755)
    return launcher


BASE_PARAMS = dict(
    modem_a_serial   = "OA-2-1",
    modem_b_serial   = "OA-2-2",
    v_radial_m_s     = 0.0,
    channel_gain_db  = -30.0,
    wenz_sea_state   = 3,
    range_m          = 500.0,
)


def test_single_cell_smoke(tmp_path: Path, stub_binary: Path) -> None:
    out_dir = tmp_path / "out"
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [42]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out_dir,
        duration_s    = 2.0,        # stub exits on SIGTERM
        parallel      = 1,
    )
    results = sweep.run()
    assert len(results) == 1
    r = results[0]
    assert r.ok, f"cell failed: {r.error}, exit={r.exit_code}"
    assert r.summary_path is not None
    assert r.summary_path.name == "two_modem_base_summary.json"
    assert (r.cell_dir / "scenario.yaml").is_file()
    assert (r.cell_dir / "stdout.log").is_file()
    assert (r.cell_dir / "modem_a_tx.wav").is_file()
    assert (r.cell_dir / "modem_b_tx.wav").is_file()


def test_sweep_index_csv_has_one_row_per_cell(tmp_path: Path,
                                              stub_binary: Path) -> None:
    out_dir = tmp_path / "out"
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1, 2, 3]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out_dir,
        duration_s    = 2.0,
        parallel      = 1,
    )
    sweep.run()
    idx = out_dir / "sweep_index.csv"
    assert idx.is_file()
    with idx.open() as fp:
        rows = list(csv.DictReader(fp))
    assert len(rows) == 3
    seeds = sorted(int(r["seed"]) for r in rows)
    assert seeds == [1, 2, 3]
    for r in rows:
        assert r["ok"] == "1", f"cell {r['cell_id']} not ok: {r['error']}"


def test_sigterm_short_circuits_duration(tmp_path: Path,
                                         stub_binary: Path) -> None:
    """With ``duration_s`` well under the stub's 60 s cap, the runner
    SIGTERMs the cell and it exits cleanly."""
    out_dir = tmp_path / "out"
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [7]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out_dir,
        duration_s    = 1.0,
        sigterm_grace_s = 3.0,
    )
    results = sweep.run()
    assert results[0].ok
    assert results[0].exit_code == 0


def test_missing_binary_reports_error_without_raising(tmp_path: Path) -> None:
    out_dir = tmp_path / "out"
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = tmp_path / "does_not_exist",
        out_dir       = out_dir,
        duration_s    = 1.0,
    )
    results = sweep.run()
    assert len(results) == 1
    assert not results[0].ok
    assert "binary not found" in results[0].error


def test_cell_ids_are_stable_across_runs(tmp_path: Path,
                                         stub_binary: Path) -> None:
    """Same parameter set produces the same cell_id, even on a fresh
    out_dir."""
    out_a = tmp_path / "a"
    out_b = tmp_path / "b"
    common = dict(
        template_path = TEMPLATE,
        parameters    = {"seed": [11, 22]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        duration_s    = 1.0,
    )
    a = Sweep(**common, out_dir=out_a).run()
    b = Sweep(**common, out_dir=out_b).run()
    a_ids = sorted(r.cell_id for r in a)
    b_ids = sorted(r.cell_id for r in b)
    assert a_ids == b_ids


# ---------------------------------------------------------------------------
# Watchdog (max_cell_runtime_s)
# ---------------------------------------------------------------------------

def test_watchdog_fires_when_stub_ignores_sigterm(tmp_path: Path,
                                                  stubborn_binary: Path) -> None:
    """``duration_s=inf`` + watchdog: a stuck stub that ignores SIGTERM
    is killed by the watchdog and reported as failed with the watchdog
    reason."""
    t0 = time.monotonic()
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = stubborn_binary,
        out_dir       = tmp_path / "out",
        duration_s    = float("inf"),
        max_cell_runtime_s = 0.8,
        sigterm_grace_s    = 0.5,
        poll_interval_s    = 0.1,
        progress           = False,
    )
    results = sweep.run()
    elapsed = time.monotonic() - t0

    assert len(results) == 1
    r = results[0]
    assert not r.ok, "watchdog-killed cell must report failure"
    assert "exceeded max_cell_runtime_s" in r.error, r.error
    # Watchdog 0.8 + SIGTERM grace 0.5 + SIGKILL wait 2.0 < 4 s.
    assert elapsed < 5.0, f"watchdog took too long: {elapsed:.1f}s"


def test_watchdog_does_not_fire_when_cell_finishes_naturally(
        tmp_path: Path, stub_binary: Path) -> None:
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = tmp_path / "out",
        duration_s    = 1.0,
        max_cell_runtime_s = 60.0,
        progress           = False,
    )
    r = sweep.run()[0]
    assert r.ok
    assert "watchdog" not in r.error.lower()
    assert "exceeded" not in r.error


def test_stop_condition_can_fire_before_duration(tmp_path: Path,
                                                  stub_binary: Path) -> None:
    """A ``stop_condition`` that returns True ends the cell early without
    the watchdog firing."""
    fired = []

    def stop_after_two_polls(handle):
        fired.append(time.monotonic())
        return len(fired) >= 3

    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = tmp_path / "out",
        duration_s    = 30.0,
        max_cell_runtime_s = 60.0,
        stop_condition = stop_after_two_polls,
        poll_interval_s = 0.1,
        progress        = False,
    )
    t0 = time.monotonic()
    r = sweep.run()[0]
    elapsed = time.monotonic() - t0
    assert r.ok
    assert elapsed < 5.0          # stop_condition shortened the cell
    assert len(fired) >= 3


# ---------------------------------------------------------------------------
# Progress logging
# ---------------------------------------------------------------------------

def test_progress_emits_one_line_per_cell_transition(tmp_path: Path,
                                                     stub_binary: Path) -> None:
    buf = io.StringIO()
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1, 2, 3]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = tmp_path / "out",
        duration_s    = 0.5,
        progress      = True,
        progress_stream = buf,
    )
    sweep.run()

    text = buf.getvalue()
    # Sweep-level lines
    assert re.search(r"\[sweep\] starting 3 cell\(s\)", text), text
    assert re.search(r"\[sweep\] complete: 3 ok, 0 fail", text), text
    # One start + one done per cell.
    starts = re.findall(r"\[sweep\] cell \w+ starting", text)
    dones  = re.findall(r"\[sweep\] cell \w+ ok in",    text)
    assert len(starts) == 3, text
    assert len(dones)  == 3, text


def test_progress_includes_axis_param_values(tmp_path: Path,
                                             stub_binary: Path) -> None:
    """Axis-parameter values appear in the start line so the operator can
    distinguish cells without correlating cell_id."""
    buf = io.StringIO()
    Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [777]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = tmp_path / "out",
        duration_s    = 0.5,
        progress_stream = buf,
    ).run()
    assert "seed=777" in buf.getvalue()


def test_progress_marks_watchdog_failure_as_FAIL(tmp_path: Path,
                                                  stubborn_binary: Path) -> None:
    buf = io.StringIO()
    Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = stubborn_binary,
        out_dir       = tmp_path / "out",
        duration_s    = float("inf"),
        max_cell_runtime_s = 0.5,
        sigterm_grace_s    = 0.3,
        progress_stream    = buf,
    ).run()
    text = buf.getvalue()
    assert "FAIL" in text, text
    assert "exceeded max_cell_runtime_s" in text, text
    assert "0 ok, 1 fail" in text, text


def test_progress_disabled_emits_nothing(tmp_path: Path,
                                          stub_binary: Path) -> None:
    buf = io.StringIO()
    Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = tmp_path / "out",
        duration_s    = 0.5,
        progress      = False,
        progress_stream = buf,
    ).run()
    assert buf.getvalue() == ""


def test_retries_attempted_on_failure(tmp_path: Path) -> None:
    """With ``retries > 0`` a permanently failing cell is reattempted that
    many extra times before being marked failed."""
    buf = io.StringIO()
    results = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = tmp_path / "does_not_exist",  # always fails
        out_dir       = tmp_path / "out",
        duration_s    = 1.0,
        retries       = 2,
        progress_stream = buf,
    ).run()
    text = buf.getvalue()
    assert not results[0].ok
    # Two retry-progress lines (attempts 2 and 3 out of 3 total).
    retries = re.findall(r"\[sweep\] cell \w+ retry 2/3", text)
    assert len(retries) == 1, text
    retries = re.findall(r"\[sweep\] cell \w+ retry 3/3", text)
    assert len(retries) == 1, text


def test_retries_zero_makes_one_attempt(tmp_path: Path) -> None:
    buf = io.StringIO()
    Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = tmp_path / "does_not_exist",
        out_dir       = tmp_path / "out",
        duration_s    = 1.0,
        retries       = 0,
        progress_stream = buf,
    ).run()
    assert "retry" not in buf.getvalue()


def test_retries_stop_after_first_success(tmp_path: Path,
                                          stub_binary: Path) -> None:
    """A passing cell is not retried."""
    buf = io.StringIO()
    results = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = tmp_path / "out",
        duration_s    = 0.5,
        retries       = 3,
        progress_stream = buf,
    ).run()
    assert results[0].ok
    assert "retry" not in buf.getvalue()


def test_progress_handles_infinite_duration_with_no_watchdog(
        tmp_path: Path, stub_binary: Path) -> None:
    """If ``duration_s=inf`` and no watchdog is set, the start line says
    the budget is unknown rather than printing 'inf' or failing
    ``math.isfinite``."""
    buf = io.StringIO()
    # stop_condition is what actually ends the cell.
    Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = tmp_path / "out",
        duration_s    = float("inf"),
        stop_condition = lambda h: True,
        progress_stream = buf,
    ).run()
    assert "budget unknown" in buf.getvalue(), buf.getvalue()
