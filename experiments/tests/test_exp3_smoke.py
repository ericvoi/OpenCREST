"""End-to-end smoke test for the Exp 3 analysis pipeline.

Synthesises fake per-cell artifacts (scenario.yaml, tx_log.csv,
modem_b_cdc.log, cell_meta.json, summary.json) and exercises the real
``analyse``, CSV writers, and figure renderer.

Each cell is one (range, sea_state, seed) tuple; per-cell PER is
``(n_total - n_received) / n_total``; pooling happens across seeds.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from experiments.exp3_janus_per import (
    analyse,
    analyse_cell,
    render_per_figure,
    write_per_vs_range_csv,
    write_pooled_per_csv,
    write_processing_table_csv,
)


# ---------------------------------------------------------------------------
# Fixture: a fully-formed fake cell directory
# ---------------------------------------------------------------------------

def _synthesize_cell(out_dir: Path,
                     *,
                     range_m:    float,
                     sea_state:  int,
                     seed:       int,
                     received_seqs: set[int],
                     n_packets:  int = 15,
                     cadence_s:  float = 5.0,
                     spawn_ns:   int = 1_000_000_000,
                     ) -> Path:
    """Build one cell's worth of analysis inputs."""
    cell_dir = out_dir / f"cell_R{int(range_m)}_ss{sea_state}_seed{seed}"
    cell_dir.mkdir(parents=True, exist_ok=True)

    (cell_dir / "scenario.yaml").write_text(
        f"name: exp3_janus_per\n"
        f"description: synthetic smoke fixture R={range_m}m ss={sea_state} seed={seed}\n"
    )

    # tx_log.csv: one request per packet at the configured cadence.
    tx_rows = []
    for i in range(n_packets):
        req_ns  = spawn_ns + int((5.0 + i * cadence_s) * 1e9)
        tx_rows.append((i, req_ns, f"PROBE {i:03d}"))
    with (cell_dir / "tx_log.csv").open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["request_id", "request_ts_ns", "payload"])
        for row in tx_rows:
            w.writerow(row)

    # modem_b_cdc.log: include the full multi-line decode block for each
    # received seq, plus noise lines that must not match SMS_PROBE_RE.
    cdc_lines: list[str] = []
    for seq in sorted(received_seqs):
        cdc_lines.extend([
            "Received a new message at 237s",
            "Mobility flag: 1",
            "Schedule flag: 1",
            "Tx/Rx flag: 1",
            "Forwarding capability: 0",
            "Class user i.d.: 11",
            "Application type: 1",
            "SNR: 218.95",
            "Relative speed: -0.342 m/s",
            "Message length (bits): 80",
            "Sender i.d.: 73",
            "Destination i.d.: 73",
            "Coding: 0",
            "Encryption: 0",
            f"SMS: PROBE {seq:03d}",
        ])
    fake_ns = spawn_ns
    with (cell_dir / "modem_b_cdc.log").open("w") as fp:
        for ln in cdc_lines:
            fake_ns += 1_000_000
            fp.write(f"[{fake_ns} ns] {ln}\n")

    # cell_meta.json
    (cell_dir / "cell_meta.json").write_text(json.dumps(dict(
        simulator_spawn_ns = spawn_ns,
        range_m            = range_m,
        sea_state          = sea_state,
        seed               = seed,
        cadence_s          = cadence_s,
        packets_per_cell   = n_packets,
    )))

    summary = dict(
        scenario_name = "exp3_janus_per",
        random_seed   = seed,
        duration_s    = float(n_packets * cadence_s + 10),
        modems        = ["modem_a", "modem_b"],
        processing_time = dict(count=n_packets, mean_us=210.0, p50_us=190,
                               p95_us=320, p99_us=370, max_us=480,
                               underrun_count=0),
        channel_engine  = dict(rx_ring_underruns=0, fw_rx_underruns=0,
                               tx_packets_total=n_packets,
                               rx_packets_total=len(received_seqs)),
    )
    (cell_dir / "exp3_janus_per_summary.json").write_text(json.dumps(summary))
    return cell_dir


# ---------------------------------------------------------------------------
# Single-cell shape tests
# ---------------------------------------------------------------------------

def test_analyse_cell_counts_per_correctly(tmp_path: Path) -> None:
    _synthesize_cell(tmp_path, range_m=500.0, sea_state=3, seed=0,
                     received_seqs={0, 1, 2, 10, 11, 12},
                     n_packets=15)
    cell_dir = tmp_path / "cell_R500_ss3_seed0"
    a = analyse_cell(cell_dir)
    assert a is not None
    assert a.result.range_m    == 500.0
    assert a.result.sea_state  == 3
    assert a.result.seed       == 0
    assert a.result.n_total    == 15
    assert a.result.n_received == 6
    assert a.result.per        == 9 / 15


def test_analyse_cell_returns_none_for_missing_tx_log(tmp_path: Path) -> None:
    cell = tmp_path / "incomplete"
    cell.mkdir()
    (cell / "scenario.yaml").write_text("name: exp3_janus_per\n")
    assert analyse_cell(cell) is None


def test_analyse_cell_returns_none_when_meta_lacks_range_m(tmp_path: Path) -> None:
    """A cell with no range_m in cell_meta.json must be skipped."""
    cell = tmp_path / "legacy"
    cell.mkdir()
    (cell / "tx_log.csv").write_text(
        "request_id,request_ts_ns,payload\n"
        "0,1000000000,PROBE 000\n")
    (cell / "modem_b_cdc.log").write_text(("x" * 200) + "\n")
    (cell / "cell_meta.json").write_text(json.dumps(dict(sea_state=3, seed=0)))
    assert analyse_cell(cell) is None


def test_received_seqs_outside_request_range_are_ignored(tmp_path: Path) -> None:
    """A PROBE seq in modem_b_cdc.log that was never requested must not be
    counted."""
    _synthesize_cell(tmp_path, range_m=500.0, sea_state=3, seed=0,
                     received_seqs={0, 1, 99},        # 99 > n_packets=15
                     n_packets=15)
    a = analyse_cell(tmp_path / "cell_R500_ss3_seed0")
    assert a is not None
    assert a.result.n_total    == 15
    assert a.result.n_received == 2                   # only 0 and 1 requested


# ---------------------------------------------------------------------------
# Multi-cell aggregation + CSV writers
# ---------------------------------------------------------------------------

def test_per_vs_range_csv_one_row_per_cell(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    for r in (300.0, 500.0, 700.0):
        for ss in (1, 3, 5):
            for seed in range(2):
                _synthesize_cell(out_dir, range_m=r, sea_state=ss, seed=seed,
                                 received_seqs={0, 1, 2, 5, 10},
                                 n_packets=15)
    analyses = analyse(out_dir)
    assert len(analyses) == 18           # 3 ranges × 3 sea states × 2 seeds

    csv_path = out_dir / "per_vs_range.csv"
    write_per_vs_range_csv(analyses, csv_path)
    rows = list(csv.DictReader(csv_path.open()))
    assert len(rows) == 18
    assert set(rows[0].keys()) == {
        "range_m", "sea_state", "seed", "n_total", "n_received", "per",
    }
    # Rows sorted by (range_m, sea_state, seed) -> R=300, ss=1, seed=0.
    assert rows[0]["range_m"]   == "300.0"
    assert rows[0]["sea_state"] == "1"
    assert rows[0]["seed"]      == "0"
    # All PER values are in [0, 1].
    for row in rows:
        v = float(row["per"])
        assert 0.0 <= v <= 1.0


def test_pooled_csv_has_per_range_sea_state_with_ci(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    for r in (300.0, 700.0):
        for ss in (1, 3, 5):
            for seed in range(4):
                rate = {1: 0.85, 3: 0.55, 5: 0.20}[ss]
                received = {i for i in range(15)
                            if (i * 7 + seed) % 100 < int(rate * 100)}
                _synthesize_cell(out_dir, range_m=r, sea_state=ss, seed=seed,
                                 received_seqs=received, n_packets=15)
    analyses = analyse(out_dir)
    pooled_csv = out_dir / "per_vs_range_pooled.csv"
    write_pooled_per_csv(analyses, pooled_csv)
    rows = list(csv.DictReader(pooled_csv.open()))
    # 2 ranges × 3 sea states = 6 pooled rows
    assert len(rows) == 6
    assert set(rows[0].keys()) == {
        "range_m", "sea_state", "n_total", "n_received",
        "per", "per_ci_lo", "per_ci_hi",
    }
    # CI brackets PER, all in [0, 1].
    for r in rows:
        per = float(r["per"]); lo = float(r["per_ci_lo"]); hi = float(r["per_ci_hi"])
        assert 0.0 <= lo <= per <= hi <= 1.0
    # Pooled n = 4 seeds * 15 packets per cell.
    for r in rows:
        assert int(r["n_total"]) == 60


def test_processing_table_writes_one_row_per_cell(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, range_m=400.0, sea_state=1, seed=0,
                     received_seqs=set())
    _synthesize_cell(out_dir, range_m=800.0, sea_state=5, seed=0,
                     received_seqs=set())
    analyses = analyse(out_dir)
    write_processing_table_csv(analyses,
                               out_dir / "processing_table.csv")
    rows = list(csv.DictReader((out_dir / "processing_table.csv").open()))
    assert len(rows) == 2
    by_range = {float(r["range_m"]): r for r in rows}
    assert by_range[400.0]["sea_state"] == "1"
    assert by_range[400.0]["p99_us"]    == "370"
    assert by_range[800.0]["sea_state"] == "5"
    assert by_range[800.0]["max_us"]    == "480"


# ---------------------------------------------------------------------------
# Figure renderer (smoke: produces a non-trivial PDF)
# ---------------------------------------------------------------------------

def test_render_per_figure_writes_pdf(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    rng = np.random.default_rng(0)
    for range_m in (300.0, 500.0, 700.0, 900.0):
        for ss in (1, 3, 5):
            for seed in range(3):
                recv_rate = {1: 0.85, 3: 0.55, 5: 0.25}[ss]
                received = {i for i in range(15)
                            if rng.random() < recv_rate}
                _synthesize_cell(out_dir, range_m=range_m,
                                 sea_state=ss, seed=seed,
                                 received_seqs=received, n_packets=15)
    analyses = analyse(out_dir)
    pdf_path = tmp_path / "fig_janus_per.pdf"
    render_per_figure(analyses, pdf_path, sea_states=(1, 3, 5))
    assert pdf_path.is_file()
    assert pdf_path.stat().st_size > 2000


def test_render_per_figure_handles_single_state(tmp_path: Path) -> None:
    """Renders successfully when only one sea state is requested."""
    out_dir = tmp_path / "results"
    for r in (300.0, 700.0):
        _synthesize_cell(out_dir, range_m=r, sea_state=3, seed=0,
                         received_seqs={i for i in range(15) if i % 2 == 0})
    analyses = analyse(out_dir)
    pdf_path = tmp_path / "fig.pdf"
    render_per_figure(analyses, pdf_path, sea_states=(3,))
    assert pdf_path.is_file()


def test_render_per_figure_handles_single_range_point(tmp_path: Path) -> None:
    """Single (range, sea_state) point renders without ZeroDivision."""
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, range_m=500.0, sea_state=3, seed=0,
                     received_seqs={i for i in range(15) if i % 2 == 0})
    analyses = analyse(out_dir)
    pdf_path = tmp_path / "fig.pdf"
    render_per_figure(analyses, pdf_path, sea_states=(3,))
    assert pdf_path.is_file()
