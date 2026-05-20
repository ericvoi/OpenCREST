"""Tests for the metrics loader, WAV reader, and determinism fingerprint.

These three modules are tightly coupled (the determinism check uses
the wav_io + metrics_loader parsers), so test them together against a
shared synthetic cell-output fixture.
"""
from __future__ import annotations

import json
import struct
import subprocess
import sys
from pathlib import Path

import pytest

from experiments.lib import determinism, metrics_loader, wav_io
from experiments.lib.runner import Sweep


REPO = Path(__file__).resolve().parents[2]
TEMPLATE = REPO / "experiments" / "configs" / "common" / "two_modem_base.yaml.j2"
STUB     = REPO / "experiments" / "tests" / "_stub_binary.py"


BASE_PARAMS = dict(
    modem_a_serial   = "OA-2-1",
    modem_b_serial   = "OA-2-2",
    v_radial_m_s     = 0.0,
    channel_gain_db  = -30.0,
    wenz_sea_state   = 3,
    range_m          = 500.0,
)


@pytest.fixture
def stub_binary(tmp_path: Path) -> Path:
    launcher = tmp_path / "openCREST_stub"
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        f"exec {sys.executable} {STUB} \"$@\"\n",
    )
    launcher.chmod(0o755)
    return launcher


@pytest.fixture
def stub_cell_dir(tmp_path: Path, stub_binary: Path) -> Path:
    out_dir = tmp_path / "out"
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [42]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out_dir,
        duration_s    = 1.0,
        sigterm_grace_s = 3.0,
    )
    results = sweep.run()
    assert results[0].ok, results[0].error
    return results[0].cell_dir


# ---------------------------------------------------------------------------
# wav_io
# ---------------------------------------------------------------------------

def test_wav_io_reads_stub_minimal_wav(stub_cell_dir: Path) -> None:
    wav = wav_io.read_wav(stub_cell_dir / "modem_a_tx.wav")
    assert wav.rate == 500_000
    assert wav.nchannels == 1
    assert wav.sampwidth == 2
    assert wav.samples.size == 1
    assert wav.samples.dtype.kind == "f"


def test_wav_io_tolerates_zero_data_size(tmp_path: Path) -> None:
    """Writer-interrupted WAV: data chunk size set to 0; reader should
    fall back to reading until EOF and parse without raising."""
    path = tmp_path / "ragged.wav"
    sample_rate = 500_000
    samples = b"\x00\x00\x10\x00\x20\x00"
    fmt = struct.pack("<4sIHHIIHH",
                      b"fmt ", 16, 1, 1, sample_rate,
                      sample_rate * 2, 2, 16)
    data = struct.pack("<4sI", b"data", 0) + samples   # size=0 -> tolerant path
    riff = struct.pack("<4sI4s", b"RIFF", 0, b"WAVE")
    path.write_bytes(riff + fmt + data)

    wav = wav_io.read_wav(path)
    assert wav.samples.size == 3


# ---------------------------------------------------------------------------
# metrics_loader
# ---------------------------------------------------------------------------

def test_load_summary_returns_dict(stub_cell_dir: Path) -> None:
    summary = metrics_loader.load_summary(
        stub_cell_dir / "two_modem_base_summary.json")
    assert summary["scenario_name"] == "two_modem_base"
    assert summary["random_seed"] == 42
    assert "processing_time" in summary
    assert "channel_engine"  in summary


def test_load_events_returns_dataframe(stub_cell_dir: Path) -> None:
    df = metrics_loader.load_events(stub_cell_dir / "modem_a_events.jsonl")
    assert len(df) == 3
    assert set(df["direction"].unique()) <= {"Tx", "Rx"}
    assert "duration_ns" in df.columns
    assert (df["duration_ns"] > 0).all()


def test_load_sweep_long_form(tmp_path: Path, stub_binary: Path) -> None:
    out_dir = tmp_path / "out"
    sweep = Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [1, 2]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out_dir,
        duration_s    = 1.0,
    )
    sweep.run()
    df = metrics_loader.load_sweep(out_dir)
    # 2 cells x 2 modems x 3 events = 12 rows
    assert len(df) == 12
    assert set(df["random_seed"].unique()) == {1, 2}
    assert set(df["modem_id"].unique()) == {"modem_a", "modem_b"}


def test_load_sweep_index_csv(tmp_path: Path, stub_binary: Path) -> None:
    out_dir = tmp_path / "out"
    Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [10]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out_dir,
        duration_s    = 1.0,
    ).run()
    idx = metrics_loader.load_sweep_index(out_dir)
    assert len(idx) == 1
    assert int(idx.iloc[0]["seed"]) == 10
    assert int(idx.iloc[0]["ok"]) == 1


def test_load_cdc_parses_timestamp_prefix(tmp_path: Path) -> None:
    path = tmp_path / "modem_a_cdc.log"
    path.write_text(
        "[1000000000 ns] Range: 487.20m\n"
        "[1000000125 ns] Received: PROBE 0042\n"
    )
    df = metrics_loader.load_cdc(path)
    assert len(df) == 2
    assert df.iloc[0]["timestamp_ns"] == 1_000_000_000
    assert "PROBE 0042" in df.iloc[1]["line"]


# ---------------------------------------------------------------------------
# determinism
# ---------------------------------------------------------------------------

def test_repeat_run_is_deterministic_under_stub(tmp_path: Path,
                                                stub_binary: Path) -> None:
    """The stub binary writes byte-identical artifacts on every run; the
    determinism report should be all-PASS. This validates the wiring of
    the fingerprint comparator end-to-end."""
    sweep = lambda out: Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [99]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out,
        duration_s    = 1.0,
    ).run()[0].cell_dir

    cell_a = sweep(tmp_path / "a")
    cell_b = sweep(tmp_path / "b")
    report = determinism.compare_cells(cell_a, cell_b)
    assert report.ok, "\n" + report.to_text()
    # Ensure we actually compared something interesting.
    arts = [r.artifact for r in report.results]
    assert any(a.startswith("wav:")     for a in arts)
    assert any(a.startswith("events:")  for a in arts)
    assert any(a.startswith("summary:") for a in arts)


def test_summary_fingerprint_excludes_timestamps(tmp_path: Path) -> None:
    """Two summaries differing only in started_at/ended_at must compare equal
    under fingerprint_summary."""
    base = dict(
        scenario_name="x", scenario_path="x.yaml", random_seed=1,
        duration_s=1.0, modems=["modem_a"],
        processing_time=dict(count=10, mean_us=100.0, p50_us=90,
                             p95_us=180, p99_us=200, max_us=250,
                             underrun_count=0),
        channel_engine=dict(rx_ring_underruns=0, fw_rx_underruns=0,
                            tx_packets_total=10, rx_packets_total=10),
        log_files=dict(events=[], cdc=[], wav=[]),
    )
    a = dict(base, started_at="2026-01-01T00:00:00Z",
             ended_at="2026-01-01T00:00:01Z")
    b = dict(base, started_at="2099-12-31T23:59:59Z",
             ended_at="2099-12-31T23:59:60Z")
    pa, pb = tmp_path / "a.json", tmp_path / "b.json"
    pa.write_text(json.dumps(a))
    pb.write_text(json.dumps(b))
    assert determinism.fingerprint_summary(pa) == determinism.fingerprint_summary(pb)


def test_events_fingerprint_excludes_wall_clock(tmp_path: Path) -> None:
    """Events differing only in start_ns/end_ns must fingerprint identically."""
    a = tmp_path / "a.jsonl"
    b = tmp_path / "b.jsonl"
    common = dict(modem_id="modem_a", direction="Tx",
                  sample_count=100, sequence_id=0)
    a.write_text(json.dumps({**common, "start_ns": 100, "end_ns": 200}) + "\n")
    b.write_text(json.dumps({**common, "start_ns": 999, "end_ns": 1234}) + "\n")
    assert determinism.fingerprint_events(a) == determinism.fingerprint_events(b)


def test_cdc_fingerprint_strips_host_timestamps(tmp_path: Path) -> None:
    a = tmp_path / "a.log"
    b = tmp_path / "b.log"
    a.write_text("[1 ns] hello\n[2 ns] world\n")
    b.write_text("[999999 ns] hello\n[1000000 ns] world\n")
    assert determinism.fingerprint_cdc(a) == determinism.fingerprint_cdc(b)


def test_determinism_report_text_is_readable(tmp_path: Path,
                                              stub_binary: Path) -> None:
    sweep = lambda out: Sweep(
        template_path = TEMPLATE,
        parameters    = {"seed": [77]},
        extra_params  = BASE_PARAMS,
        binary        = stub_binary,
        out_dir       = out,
        duration_s    = 1.0,
    ).run()[0].cell_dir
    report = determinism.compare_cells(
        sweep(tmp_path / "a"),
        sweep(tmp_path / "b"),
    )
    text = report.to_text()
    assert "determinism:" in text
    assert text.count("PASS") >= 1
