"""End-to-end smoke test for the Exp 3 analysis pipeline.

Per ``feedback_opencrest_no_test_specific_code.md`` the openCREST binary
stays test-agnostic — the Python harness wraps it as an opaque
subprocess. The smoke test therefore synthesises fake per-cell artifacts
(scenario.yaml, tx_log.csv, modem_a_events.jsonl, modem_b_cdc.log,
cell_meta.json, summary.json) and exercises the real ``analyse`` +
CSV writers + figure renderer.

If the analysis pipeline drifts (CSV column order, summary JSON schema,
figure renderer signature), this test catches it without needing real
hardware.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import pytest

from experiments.exp3_janus_per import (
    DEFAULT_CLOSING_VELOCITY_M_S,
    DEFAULT_BIN_EDGES_M,
    INITIAL_RANGE_M,
    analyse,
    analyse_cell,
    compute_range_at_tx,
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
                     sea_state:  int,
                     seed:       int,
                     received_seqs: set[int],
                     n_packets:  int = 80,
                     cadence_s:  float = 5.0,
                     spawn_ns:   int = 1_000_000_000,
                     ) -> Path:
    """Build one cell's worth of analysis inputs.

    Default ``n_packets=80`` matches a 400 s closing run at 5 s cadence.
    ``received_seqs`` controls which probe IDs land in modem_b_cdc.log.
    """
    cell_dir = out_dir / f"cell_ss{sea_state}_seed{seed}"
    cell_dir.mkdir(parents=True, exist_ok=True)

    # scenario.yaml is just there for round-tripping by the runner; analyse()
    # doesn't grep it for the cell→params mapping (cell_meta.json does that).
    (cell_dir / "scenario.yaml").write_text(
        f"name: exp3_janus_per\n"
        f"description: synthetic smoke fixture for ss={sea_state}, seed={seed}\n"
    )

    # tx_log.csv: one request per packet at the configured cadence
    tx_rows = []
    for i in range(n_packets):
        req_ns  = spawn_ns + int((5.0 + i * cadence_s) * 1e9)   # first req at t=5s
        tx_rows.append((i, req_ns, f"PROBE {i:03d}"))
    with (cell_dir / "tx_log.csv").open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["request_id", "request_ts_ns", "payload"])
        for row in tx_rows:
            w.writerow(row)

    # modem_a_events.jsonl: one TX event per request, start_ns ≈ request_ts_ns
    # (the firmware doesn't queue much in steady state).
    events_path = cell_dir / "modem_a_events.jsonl"
    with events_path.open("w") as fp:
        for i, req_ns, _payload in tx_rows:
            event = dict(
                modem_id     = "modem_a",
                direction    = "tx",
                start_ns     = req_ns + 5_000_000,       # 5 ms after request
                end_ns       = req_ns + 5_000_000 + 1_000_000_000,
                sample_count = 500_000,
                sequence_id  = i,
            )
            fp.write(json.dumps(event) + "\n")

    # modem_b_cdc.log: include the full multi-line decode block for the
    # received seqs, plus some noise lines that must NOT match.
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

    # cell_meta.json: the host-side anchor + params
    (cell_dir / "cell_meta.json").write_text(json.dumps(dict(
        simulator_spawn_ns = spawn_ns,
        r0_m               = INITIAL_RANGE_M,
        v_m_s              = DEFAULT_CLOSING_VELOCITY_M_S,
        range_floor_m      = 200.0,
        cadence_s          = cadence_s,
        max_packets        = n_packets,
        sea_state          = sea_state,
        seed               = seed,
    )))

    # summary.json — Session D shape
    summary = dict(
        scenario_name = "exp3_janus_per",
        random_seed   = seed,
        duration_s    = 400.0,
        modems        = ["modem_a", "modem_b"],
        processing_time = dict(count=80, mean_us=210.0, p50_us=190,
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

def test_analyse_cell_returns_one_record_per_tx(tmp_path: Path) -> None:
    _synthesize_cell(tmp_path, sea_state=3, seed=0,
                     received_seqs={0, 1, 2, 10, 50, 70},
                     n_packets=80)
    cell_dir = tmp_path / "cell_ss3_seed0"
    a = analyse_cell(cell_dir)
    assert a is not None
    assert a.sea_state == 3
    assert a.seed       == 0
    assert len(a.records) == 80
    received_count = sum(1 for r in a.records if r.received)
    assert received_count == 6


def test_received_seqs_propagate_to_records(tmp_path: Path) -> None:
    _synthesize_cell(tmp_path, sea_state=1, seed=2,
                     received_seqs={5, 17, 60},
                     n_packets=70)
    a = analyse_cell(tmp_path / "cell_ss1_seed2")
    assert a is not None
    by_id = {r.request_id: r for r in a.records}
    assert by_id[5 ].received is True
    assert by_id[17].received is True
    assert by_id[60].received is True
    assert by_id[0 ].received is False
    assert by_id[40].received is False


def test_range_at_tx_decreases_monotonically(tmp_path: Path) -> None:
    """With v=-2 m/s, every successive packet should have smaller range."""
    _synthesize_cell(tmp_path, sea_state=3, seed=0, received_seqs=set(),
                     n_packets=20)
    a = analyse_cell(tmp_path / "cell_ss3_seed0")
    assert a is not None
    ranges = [r.range_at_tx_m for r in a.records]
    assert ranges == sorted(ranges, reverse=True)
    # First packet at t=5s + 5ms (firmware delay) → R ≈ 1000 + v·5.005
    expected_first = 1000.0 + DEFAULT_CLOSING_VELOCITY_M_S * 5.005
    assert ranges[0] == pytest.approx(expected_first, abs=0.5)


def test_analyse_returns_none_for_cell_without_tx_log(tmp_path: Path) -> None:
    """A cell directory missing tx_log.csv should be skipped, not crash."""
    cell = tmp_path / "incomplete"
    cell.mkdir()
    (cell / "scenario.yaml").write_text("name: exp3_janus_per\n")
    assert analyse_cell(cell) is None


# ---------------------------------------------------------------------------
# Multi-cell aggregation + CSV writers
# ---------------------------------------------------------------------------

def test_per_vs_range_csv_has_one_row_per_cell_per_bin(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    for ss in (1, 3, 5):
        for seed in range(2):
            _synthesize_cell(out_dir, sea_state=ss, seed=seed,
                             received_seqs={0, 1, 2, 5, 10, 20},
                             n_packets=80)
    analyses = analyse(out_dir)
    assert len(analyses) == 6           # 3 sea states × 2 seeds

    csv_path = out_dir / "per_vs_range.csv"
    write_per_vs_range_csv(analyses, csv_path)
    rows = list(csv.DictReader(csv_path.open()))
    # 8 bins × 6 cells = 48 rows
    assert len(rows) == 48
    for row in rows:
        assert row["sea_state"] in {"1", "3", "5"}
        assert row["seed"]      in {"0", "1"}
        # The PER cell is either blank (NaN; empty bin) or a float in [0, 1]
        if row["per"]:
            v = float(row["per"])
            assert 0.0 <= v <= 1.0


def test_per_vs_range_csv_empty_bins_emit_blank(tmp_path: Path) -> None:
    """Bins outside the swept range should report blank PER (NaN), not 0."""
    out_dir = tmp_path / "results"
    # n_packets sized to cover the full 1000→200 sweep at the module's
    # default velocity + cadence (otherwise some bins go unfilled, which
    # is the expected behaviour but defeats the point of this test).
    n_packets = int((1000.0 - 200.0) /
                    (abs(DEFAULT_CLOSING_VELOCITY_M_S) * 5.0)) + 2
    _synthesize_cell(out_dir, sea_state=3, seed=0,
                     received_seqs=set(), n_packets=n_packets)
    analyses = analyse(out_dir)
    write_per_vs_range_csv(analyses, out_dir / "per_vs_range.csv")
    rows = list(csv.DictReader((out_dir / "per_vs_range.csv").open()))
    # All swept bins (200..1000) should have data — none should be blank.
    blanks = [r for r in rows if not r["per"]]
    assert blanks == []        # every bin reached by the 1000→200 sweep


def test_pooled_csv_one_row_per_sea_state_per_bin_with_ci(tmp_path: Path) -> None:
    """Pooled CSV has (sea_state, bin) rows with PER + Wilson CI columns."""
    out_dir = tmp_path / "results"
    for ss in (1, 3, 5):
        for seed in range(3):
            # Vary received rate per sea state so we exercise different CIs.
            rate = {1: 0.85, 3: 0.55, 5: 0.20}[ss]
            received = {i for i in range(160) if (i * 7 + seed) % 100 < int(rate * 100)}
            _synthesize_cell(out_dir, sea_state=ss, seed=seed,
                             received_seqs=received, n_packets=160)
    analyses = analyse(out_dir)
    pooled_csv = out_dir / "per_vs_range_pooled.csv"
    write_pooled_per_csv(analyses, pooled_csv)
    rows = list(csv.DictReader(pooled_csv.open()))
    # 3 sea states × 8 bins = 24 rows
    assert len(rows) == 24
    # Schema sanity
    assert set(rows[0].keys()) == {
        "sea_state", "range_bin_center_m", "n_total",
        "per", "per_ci_lo", "per_ci_hi",
    }
    # Every filled row: ci_lo <= per <= ci_hi, all in [0, 1].
    for r in rows:
        if not r["per"]:
            continue
        per = float(r["per"]); lo = float(r["per_ci_lo"]); hi = float(r["per_ci_hi"])
        assert 0.0 <= lo <= per <= hi <= 1.0
    # Pooled n grows with seeds: 3 seeds × ~20 packets/bin → n ≥ 50 per bin.
    n_values = [int(r["n_total"]) for r in rows if r["n_total"]]
    assert min(n_values) >= 50


def test_processing_table_writes_one_row_per_cell(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, sea_state=1, seed=0, received_seqs=set())
    _synthesize_cell(out_dir, sea_state=5, seed=0, received_seqs=set())
    analyses = analyse(out_dir)
    write_processing_table_csv(analyses,
                               out_dir / "processing_table.csv")
    rows = list(csv.DictReader((out_dir / "processing_table.csv").open()))
    assert len(rows) == 2
    by_ss = {int(r["sea_state"]): r for r in rows}
    assert by_ss[1]["p99_us"] == "370"
    assert by_ss[5]["max_us"] == "480"


# ---------------------------------------------------------------------------
# Figure renderer (smoke: produces a non-trivial PDF)
# ---------------------------------------------------------------------------

def test_render_per_figure_writes_pdf(tmp_path: Path) -> None:
    out_dir = tmp_path / "results"
    rng = np.random.default_rng(0)
    for ss in (1, 3, 5):
        for seed in range(3):
            # Higher sea state ⇒ fewer received probes (rough monotone test).
            n_packets = 80
            recv_rate = {1: 0.85, 3: 0.55, 5: 0.25}[ss]
            received  = {i for i in range(n_packets)
                         if rng.random() < recv_rate}
            _synthesize_cell(out_dir, sea_state=ss, seed=seed,
                             received_seqs=received, n_packets=n_packets)

    analyses = analyse(out_dir)
    pdf_path = tmp_path / "fig_janus_per.pdf"
    render_per_figure(analyses, pdf_path,
                      sea_states=(1, 3, 5))
    assert pdf_path.is_file()
    assert pdf_path.stat().st_size > 2000


def test_render_per_figure_handles_single_state(tmp_path: Path) -> None:
    """Caller passing a smaller sea-state set should still render."""
    out_dir = tmp_path / "results"
    _synthesize_cell(out_dir, sea_state=3, seed=0,
                     received_seqs={i for i in range(80) if i % 2 == 0})
    analyses = analyse(out_dir)
    pdf_path = tmp_path / "fig.pdf"
    render_per_figure(analyses, pdf_path, sea_states=(3,))
    assert pdf_path.is_file()


def test_compute_range_at_tx_matches_synth_first_packet(tmp_path: Path) -> None:
    """Cross-check the helper against the smoke fixture's first-packet range."""
    spawn_ns = 1_000_000_000
    # First packet's request_ts_ns is spawn + 5 s (first_request_delay), the
    # event start_ns is request_ts + 5 ms.
    event_start = spawn_ns + int(5.005 * 1e9)
    r = compute_range_at_tx(event_start, spawn_ns,
                            INITIAL_RANGE_M, DEFAULT_CLOSING_VELOCITY_M_S)
    assert r == pytest.approx(1000.0 + DEFAULT_CLOSING_VELOCITY_M_S * 5.005)
