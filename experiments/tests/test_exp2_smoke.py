"""End-to-end smoke test for the Exp 2 analysis pipeline.

Per ``feedback_opencrest_no_test_specific_code.md`` the openCREST binary
stays test-agnostic — the Python harness wraps it as an opaque
subprocess. The smoke test therefore synthesizes a fake cell directory
(scenario.yaml, requests.jsonl, summary JSON) for one config and
exercises the real ``analyse`` + CSV writers + violin renderer.

If the analysis pipeline drifts (CSV column order, summary JSON schema,
violin labels), this test catches it without needing real hardware.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from experiments.exp2_ranging_accuracy import (
    CONFIGURED_RANGE_M,
    analyse,
    render_violin,
    write_processing_table_csv,
    write_ranging_results_csv,
)


def _synthesize_cell(out_dir: Path,
                     config_id: str,
                     *,
                     range_readouts: list[float],
                     processing: dict | None = None,
                     ) -> Path:
    """Write one cell's worth of analysis-input artifacts.

    ``range_readouts`` is one entry per request: a ``float`` produces a
    successful response with that range, ``None`` produces a timeout.
    """
    cell_dir = out_dir / f"cell_{config_id}"
    cell_dir.mkdir(parents=True, exist_ok=True)

    # scenario.yaml — analysis uses the ``name:`` line to map cell -> config
    (cell_dir / "scenario.yaml").write_text(
        f"name: exp2_ranging_{config_id}\n"
        f"description: synthetic smoke fixture\n"
    )

    # requests.jsonl — one row per request
    records = []
    base_ts = 1_000_000_000
    for i, rng in enumerate(range_readouts):
        req_ts = base_ts + i * 1_000_000_000
        if rng is None:
            rec = dict(request_id=i, request_ts_ns=req_ts,
                       response_ts_ns=None, response_text=None,
                       range_m=None)
        else:
            rec = dict(
                request_id     = i,
                request_ts_ns  = req_ts,
                response_ts_ns = req_ts + 700_000_000,
                response_text  = f"Range: {rng:.2f}m",
                range_m        = float(rng),
            )
        records.append(rec)
    with (cell_dir / "requests.jsonl").open("w") as fp:
        for rec in records:
            fp.write(json.dumps(rec) + "\n")

    # summary.json — minimal Session D-shaped payload
    pt = processing or dict(
        count=1234, mean_us=190.0, p50_us=170, p95_us=290,
        p99_us=320, max_us=410, underrun_count=0,
    )
    summary_path = cell_dir / f"exp2_ranging_{config_id}_summary.json"
    summary_path.write_text(json.dumps(dict(
        scenario_name = f"exp2_ranging_{config_id}",
        random_seed   = 0,
        duration_s    = 200.0,
        modems        = ["modem_a", "modem_b"],
        processing_time = pt,
        channel_engine  = dict(rx_ring_underruns=0, fw_rx_underruns=0,
                               tx_packets_total=400, rx_packets_total=400),
    )))
    return cell_dir


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_config_a_six_responses_all_at_500(tmp_path: Path) -> None:
    """Synthesise 6 ``Range: 500.00m`` responses for config (a) and assert
    the analysis pipeline writes 6 result rows with zero error."""
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, "a",
                     range_readouts=[500.0] * 6)

    analyses = analyse(out_dir)
    assert len(analyses) == 1
    a = analyses[0]
    assert a.config_id == "a"
    assert a.num_requests == 6
    assert a.num_responses == 6
    assert a.success_rate == 1.0

    results_csv = out_dir / "ranging_results.csv"
    write_ranging_results_csv(analyses, results_csv)
    rows = list(csv.DictReader(results_csv.open()))
    assert len(rows) == 6
    for row in rows:
        assert row["config"] == "a"
        assert float(row["range_m"]) == 500.0
        assert float(row["error_m"]) == 0.0


def test_processing_table_row_per_config(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, "a", range_readouts=[500.0, 500.0])
    _synthesize_cell(out_dir, "b", range_readouts=[502.0, 498.5],
                     processing=dict(count=222, mean_us=210.0, p50_us=180,
                                     p95_us=320, p99_us=370, max_us=520,
                                     underrun_count=0))

    analyses = analyse(out_dir)
    cfg_ids = [a.config_id for a in analyses]
    assert cfg_ids == ["a", "b"]

    csv_path = out_dir / "processing_table.csv"
    write_processing_table_csv(analyses, csv_path)
    rows = list(csv.DictReader(csv_path.open()))
    assert {r["config"] for r in rows} == {"a", "b"}
    by_cfg = {r["config"]: r for r in rows}
    assert int(by_cfg["b"]["p99_us"]) == 370
    assert int(by_cfg["b"]["max_us"]) == 520
    assert int(by_cfg["b"]["underruns"]) == 0


def test_timeouts_are_recorded_but_excluded_from_error_stats(
        tmp_path: Path) -> None:
    """A timed-out request shows up in the CSV (blank fields) but doesn't
    contribute to the error array used by the violin plot."""
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, "a",
                     range_readouts=[500.0, None, 501.0, None])

    a = analyse(out_dir)[0]
    assert a.num_requests == 4
    assert a.num_responses == 2
    assert a.success_rate == 0.5
    errs = a.errors_m
    assert errs.size == 2
    assert sorted(errs.tolist()) == [0.0, 1.0]

    csv_path = out_dir / "ranging_results.csv"
    write_ranging_results_csv([a], csv_path)
    rows = list(csv.DictReader(csv_path.open()))
    assert len(rows) == 4
    # The two timeouts have empty range_m / error_m / latency.
    timeouts = [r for r in rows if r["range_m"] == ""]
    assert len(timeouts) == 2
    for t in timeouts:
        assert t["error_m"] == ""
        assert t["response_latency_ms"] == ""


def test_violin_renders_pdf(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, "a", range_readouts=[500.0, 500.5])
    _synthesize_cell(out_dir, "b", range_readouts=[501.0, 499.5])
    _synthesize_cell(out_dir, "c", range_readouts=[498.0, 502.0])
    _synthesize_cell(out_dir, "d", range_readouts=[497.5, 503.5])

    analyses = analyse(out_dir)
    assert [a.config_id for a in analyses] == ["a", "b", "c", "d"]

    pdf_path = tmp_path / "fig_ranging.pdf"
    render_violin(analyses, pdf_path)
    assert pdf_path.is_file()
    assert pdf_path.stat().st_size > 1000


def test_outlier_excluded_from_violin_inlier_stats_but_kept_in_csv(
        tmp_path: Path) -> None:
    """An outlier measurement (|err| above threshold) is preserved in the
    CSV (with is_outlier=1), excluded from ``errors_inlier``, listed by
    ``outliers``, and dropped from the violin distribution."""
    out_dir = tmp_path / "results"
    # 4 in-range responses + 1 wildly-off outlier at 319 m.
    _synthesize_cell(out_dir, "d",
                     range_readouts=[500.5, 499.5, 501.0, 319.15, 500.2])

    analyses = analyse(out_dir)
    a = analyses[0]
    threshold = 50.0

    # Raw errors still see all 5.
    assert a.errors_m.size == 5

    # Inlier filter drops only the 319 m measurement.
    inliers = a.errors_inlier(threshold)
    assert inliers.size == 4
    assert all(abs(e) <= threshold for e in inliers)

    # outliers() returns the dropped record(s).
    outliers = a.outliers(threshold)
    assert len(outliers) == 1
    assert outliers[0].range_m == pytest.approx(319.15)

    # CSV writes all 5 rows, flags the outlier with is_outlier=1.
    csv_path = out_dir / "ranging_results.csv"
    write_ranging_results_csv([a], csv_path,
                              outlier_threshold_m=threshold)
    rows = list(csv.DictReader(csv_path.open()))
    assert len(rows) == 5
    outlier_rows = [r for r in rows if r["is_outlier"] == "1"]
    assert len(outlier_rows) == 1
    assert outlier_rows[0]["range_m"] == "319.150"
    inlier_rows = [r for r in rows if r["is_outlier"] == "0"]
    assert len(inlier_rows) == 4


def test_analyse_skips_cells_missing_requests_jsonl(tmp_path: Path) -> None:
    """A directory with scenario.yaml but no requests.jsonl produces an
    analysis with zero records — not a crash."""
    out_dir = tmp_path / "results"
    cell = out_dir / "stub_a"
    cell.mkdir(parents=True)
    (cell / "scenario.yaml").write_text("name: exp2_ranging_a\n")

    analyses = analyse(out_dir)
    assert len(analyses) == 1
    assert analyses[0].num_requests == 0
    assert analyses[0].errors_m.size == 0
