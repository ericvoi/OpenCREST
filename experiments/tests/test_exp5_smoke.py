"""Pure-analysis and plumbing tests for exp5 (quasi-static BER vs SNR).

No hardware, no simulator: the firmware report parser, the binomial
pooling, the across-snapshot statistics, the per-cell artifact loader,
the figure, and the --analysis-only entry point.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest

from experiments.exp5_quasistatic_ber import (CellBer, analyse_cell,
                                              gain_stats, main,
                                              parse_eval_reports,
                                              pool_by_snapshot,
                                              render_ber_figure)


# ---------------------------------------------------------------------------
# Firmware report parsing
# ---------------------------------------------------------------------------

FIRMWARE_LINES = [
    "[123456789 ns] Evaluation Message:",
    "[123456790 ns] Uncoded BER: 12/800, 1.500%",
    "[123456791 ns] Coded BER: 0/384, 0.000%",
    "[123456999 ns] some unrelated status line",
    "[123457000 ns] Evaluation Message:",
    "[123457001 ns] Uncoded BER: 40/800, 5.000%",
    "[123457002 ns] Coded BER: 3/384, 0.781%",
]


def test_parse_eval_reports_pairs_counts():
    reports = parse_eval_reports(FIRMWARE_LINES)
    assert len(reports) == 2
    assert (reports[0].uncoded_errors, reports[0].uncoded_bits) == (12, 800)
    assert (reports[0].coded_errors, reports[0].coded_bits) == (0, 384)
    assert (reports[1].uncoded_errors, reports[1].uncoded_bits) == (40, 800)
    assert (reports[1].coded_errors, reports[1].coded_bits) == (3, 384)


def test_parse_eval_reports_drops_unpaired_tail():
    # Log cut after the Uncoded line: no half-report emitted.
    reports = parse_eval_reports(FIRMWARE_LINES[:2])
    assert reports == []


def test_parse_eval_reports_ignores_orphan_coded_line():
    reports = parse_eval_reports(["Coded BER: 3/384, 0.781%"])
    assert reports == []


def test_uncoded_regex_does_not_feed_coded_regex():
    # "Uncoded BER" must never be misread as a Coded report.
    reports = parse_eval_reports([
        "Uncoded BER: 10/800, 1.250%",
        "Uncoded BER: 20/800, 2.500%",
        "Coded BER: 1/384, 0.260%",
    ])
    assert len(reports) == 1
    assert reports[0].uncoded_errors == 20      # latest pending wins


# ---------------------------------------------------------------------------
# Pooling + statistics
# ---------------------------------------------------------------------------

def _cell(snapshot, gain, seed, *, n_tx=10, n_reports=10,
          ue=0, ub=8000, ce=0, cb=3840) -> CellBer:
    return CellBer(snapshot=snapshot, gain_db=gain, seed=seed,
                   n_tx=n_tx, n_reports=n_reports,
                   uncoded_errors=ue, uncoded_bits=ub,
                   coded_errors=ce, coded_bits=cb)


def test_pool_by_snapshot_sums_across_seeds():
    cells = [
        _cell(0, -50.0, 0, ue=10, ub=8000),
        _cell(0, -50.0, 1, ue=30, ub=8000),
        _cell(1, -50.0, 0, ue=100, ub=8000),
    ]
    points = pool_by_snapshot(cells)
    assert len(points) == 2
    p0 = next(p for p in points if p.snapshot == 0)
    assert p0.uncoded_errors == 40
    assert p0.uncoded_bits == 16000
    assert p0.uncoded_ber == pytest.approx(40 / 16000)
    assert p0.n_tx == 20 and p0.n_reports == 20


def test_gain_stats_across_snapshots():
    cells = [
        _cell(0, -50.0, 0, ue=80, ub=8000),      # BER 1e-2
        _cell(1, -50.0, 0, ue=240, ub=8000),     # BER 3e-2
        _cell(0, -46.0, 0, ue=0, ub=8000),
        _cell(1, -46.0, 0, ue=0, ub=8000),
    ]
    stats = gain_stats(pool_by_snapshot(cells))
    assert [g.gain_db for g in stats] == [-50.0, -46.0]
    g50 = stats[0]
    assert g50.n_snapshots == 2
    assert g50.uncoded_mean == pytest.approx(2e-2)
    # Sample std of {1e-2, 3e-2}.
    assert g50.uncoded_std == pytest.approx(math.sqrt(2) * 1e-2)
    assert g50.detection_rate == pytest.approx(1.0)
    assert stats[1].uncoded_mean == pytest.approx(0.0)


def test_gain_stats_excludes_undetected_snapshots_from_ber():
    cells = [
        _cell(0, -60.0, 0, n_reports=0, ue=0, ub=0, ce=0, cb=0),
        _cell(1, -60.0, 0, n_reports=5, ue=400, ub=4000, ce=40, cb=1920),
    ]
    stats = gain_stats(pool_by_snapshot(cells))
    g = stats[0]
    assert g.n_snapshots == 1                    # only the detected one
    assert g.uncoded_mean == pytest.approx(0.1)
    # Detection rate still counts the empty snapshot's TXs.
    assert g.detection_rate == pytest.approx(5 / 20)


# ---------------------------------------------------------------------------
# Per-cell artifact loading
# ---------------------------------------------------------------------------

def _write_cell(cell_dir: Path, *, snapshot=3, gain=-52.0, seed=1,
                n_tx=2, cdc_lines=FIRMWARE_LINES) -> Path:
    cell_dir.mkdir(parents=True)
    (cell_dir / "cell_meta.json").write_text(json.dumps(
        {"snapshot": snapshot, "gain_db": gain, "seed": seed,
         "octt_file": "SYN1_snap03.octt", "eval_len_bytes": 100,
         "cadence_s": 5.0, "packets_per_cell": n_tx}))
    with (cell_dir / "tx_log.csv").open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["request_id", "request_ts_ns"])
        for i in range(n_tx):
            w.writerow([i, 1000 + i])
    (cell_dir / "modem_b_cdc.log").write_text(
        "\n".join(cdc_lines) + "\n")
    return cell_dir


def test_analyse_cell_pools_reports(tmp_path):
    cell = _write_cell(tmp_path / "cell_a")
    result = analyse_cell(cell)
    assert result is not None
    assert result.snapshot == 3
    assert result.gain_db == pytest.approx(-52.0)
    assert result.n_tx == 2
    assert result.n_reports == 2
    assert result.uncoded_errors == 52
    assert result.uncoded_bits == 1600
    assert result.coded_errors == 3
    assert result.coded_bits == 768


def test_analyse_cell_returns_none_without_meta(tmp_path):
    d = tmp_path / "empty_cell"
    d.mkdir()
    assert analyse_cell(d) is None


# ---------------------------------------------------------------------------
# Figure + analysis-only entry point
# ---------------------------------------------------------------------------

def test_render_ber_figure(tmp_path):
    cells = [
        _cell(0, -54.0, 0, ue=200, ub=8000, ce=30, cb=3840),
        _cell(1, -54.0, 0, ue=90, ub=8000, ce=2, cb=3840),
        _cell(0, -50.0, 0, ue=8, ub=8000),
        _cell(1, -50.0, 0, ue=0, ub=8000),       # zero -> plotted at floor
    ]
    out = tmp_path / "fig.pdf"
    render_ber_figure(gain_stats(pool_by_snapshot(cells)), out)
    assert out.is_file()
    assert out.stat().st_size > 1000


def test_main_analysis_only(tmp_path, capsys):
    out = tmp_path / "results"
    _write_cell(out / "cell_a", snapshot=0, gain=-52.0, seed=0)
    _write_cell(out / "cell_b", snapshot=1, gain=-52.0, seed=0)
    rc = main(["--analysis-only", "--out", str(out)])
    assert rc == 0
    for name in ("ber_cells.csv", "ber_by_snapshot.csv",
                 "ber_vs_gain.csv", "fig_ber_vs_gain.pdf"):
        assert (out / name).is_file(), name
    with (out / "ber_vs_gain.csv").open() as fp:
        rows = list(csv.DictReader(fp))
    assert len(rows) == 1
    assert float(rows[0]["gain_db"]) == pytest.approx(-52.0)
    assert int(rows[0]["n_snapshots"]) == 2


def test_main_analysis_only_empty_dir_fails(tmp_path):
    out = tmp_path / "nothing"
    out.mkdir()
    assert main(["--analysis-only", "--out", str(out)]) == 3


# ---------------------------------------------------------------------------
# --gains parsing (multiple negative values)
# ---------------------------------------------------------------------------

def test_parse_gains_handles_space_and_comma_tokens():
    from experiments.exp5_quasistatic_ber import _parse_gains
    assert _parse_gains(["-62", "-58", "-54"]) == [-62.0, -58.0, -54.0]
    assert _parse_gains(["-62,-58,-54"]) == [-62.0, -58.0, -54.0]
    assert _parse_gains(["-62,-58", "-54"]) == [-62.0, -58.0, -54.0]


def test_argparse_accepts_multiple_negative_gains(tmp_path):
    # Regression: "--gains -62 -58 -54" must not trip argparse's option
    # detection. Analysis-only doesn't consume gains, but argparse still
    # parses them, so SystemExit here would mean the nargs form regressed.
    out = tmp_path / "results"
    _write_cell(out / "cell_a", snapshot=0, gain=-52.0, seed=0)
    rc = main(["--analysis-only", "--out", str(out),
               "--gains", "-62", "-58", "-54"])
    assert rc == 0


# ---------------------------------------------------------------------------
# Scenario template
# ---------------------------------------------------------------------------

def test_template_renders_to_valid_yaml():
    import yaml

    from experiments.exp5_quasistatic_ber import TEMPLATE, _gain_tag
    from experiments.lib.scenario_template import render_template

    doc = yaml.safe_load(render_template(TEMPLATE, {
        "snapshot": 4,
        "gain_db": -52.5,
        "gain_tag": _gain_tag(-52.5),
        "seed": 7,
        "trajectory_file": "/abs/path/BCH1_snap04.octt",
        "record_dt_s": 1.0,
        "sea_state": 3,
        "modem_a_serial": "OA-2-1",
        "modem_b_serial": "OA-2-2",
        "output_directory": "/tmp/out",
    }))
    assert doc["name"] == "exp5_snap4_m52p5_seed7"
    ch = doc["channels"][0]
    assert ch["mode"] == "replay"
    assert ch["gain_db"] == pytest.approx(-52.5)
    # Messages must start inside the first interior Catmull-Rom segment.
    assert ch["replay"]["offset_s"] == pytest.approx(1.0)
    assert ch["replay"]["advance_per_message"] is False
    assert doc["noise"]["wenz_sea_state"] == 3
    assert doc["random_seed"] == 7


def test_gain_tag_is_filesystem_safe():
    from experiments.exp5_quasistatic_ber import _gain_tag
    assert _gain_tag(-52.5) == "m52p5"
    assert _gain_tag(3.0) == "p3p0"
    assert "." not in _gain_tag(-0.5)
