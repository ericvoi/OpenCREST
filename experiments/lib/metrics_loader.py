"""Load per-run summary JSON, per-modem event JSONL, and per-modem CDC logs
into long-form pandas DataFrames suitable for plotting and aggregation.

The on-disk schemas are produced by Session D's run-summary writer and
message-event log. The CDC log is produced by ``CdcConsole`` on the
Python side -- the simulator does not emit it.
"""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Iterable

import pandas as pd


_CDC_LINE_RE = re.compile(r"^\[(?P<ts>\d+) ns\] (?P<line>.*)$")


# ---------------------------------------------------------------------------
# Single-file loaders
# ---------------------------------------------------------------------------

def load_summary(path: str | Path) -> dict:
    """Read the run-summary JSON at ``path``."""
    return json.loads(Path(path).read_text())


def load_events(path: str | Path) -> pd.DataFrame:
    """Read one modem's ``*_events.jsonl`` into a DataFrame.

    Columns: ``modem_id, direction, start_ns, end_ns, sample_count, sequence_id``.
    """
    rows: list[dict] = []
    with Path(path).open() as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    df = pd.DataFrame(rows)
    if not df.empty:
        df["duration_ns"] = df["end_ns"].astype("int64") - df["start_ns"].astype("int64")
    return df


def load_cdc(path: str | Path) -> pd.DataFrame:
    """Parse a CDC log into ``[timestamp_ns, line]`` rows."""
    rows: list[dict] = []
    for raw in Path(path).read_text().splitlines():
        m = _CDC_LINE_RE.match(raw)
        if not m:
            continue
        rows.append(dict(timestamp_ns=int(m.group("ts")),
                         line=m.group("line")))
    return pd.DataFrame(rows, columns=["timestamp_ns", "line"])


# ---------------------------------------------------------------------------
# Sweep-wide loaders
# ---------------------------------------------------------------------------

def _cell_dirs(out_dir: Path) -> Iterable[Path]:
    for child in sorted(out_dir.iterdir()):
        if child.is_dir() and (child / "scenario.yaml").is_file():
            yield child


def load_sweep(out_dir: str | Path) -> pd.DataFrame:
    """Walk a sweep output directory and return a long-form DataFrame: one
    row per ``(cell_id, modem_id, direction, sequence_id)`` message event,
    annotated with the cell's summary-level scalars (random_seed,
    p99_us, underrun_count, tx_packets_total, ...).

    Empty cells (no events) still get one row with NaNs for event fields,
    so a sweep that produced zero messages doesn't silently disappear.
    """
    out_root = Path(out_dir)
    if not out_root.is_dir():
        raise FileNotFoundError(f"sweep out_dir does not exist: {out_root}")

    all_rows: list[dict] = []
    for cell in _cell_dirs(out_root):
        summaries = list(cell.glob("*_summary.json"))
        if not summaries:
            continue
        summary = load_summary(summaries[0])
        base = dict(
            cell_id        = cell.name,
            scenario_name  = summary.get("scenario_name", ""),
            random_seed    = summary.get("random_seed", 0),
            duration_s     = summary.get("duration_s", 0.0),
            mean_us        = summary.get("processing_time", {}).get("mean_us", 0.0),
            p50_us         = summary.get("processing_time", {}).get("p50_us", 0),
            p95_us         = summary.get("processing_time", {}).get("p95_us", 0),
            p99_us         = summary.get("processing_time", {}).get("p99_us", 0),
            max_us         = summary.get("processing_time", {}).get("max_us", 0),
            underrun_count = summary.get("processing_time", {}).get("underrun_count", 0),
            tx_packets     = summary.get("channel_engine", {}).get("tx_packets_total", 0),
            rx_packets     = summary.get("channel_engine", {}).get("rx_packets_total", 0),
        )

        event_files = sorted(cell.glob("*_events.jsonl"))
        if not event_files:
            all_rows.append(dict(base, modem_id="", direction="",
                                 start_ns=pd.NA, end_ns=pd.NA,
                                 sample_count=pd.NA, sequence_id=pd.NA,
                                 duration_ns=pd.NA))
            continue
        for ev_path in event_files:
            df = load_events(ev_path)
            if df.empty:
                all_rows.append(dict(base, modem_id=ev_path.stem.replace("_events", ""),
                                     direction="", start_ns=pd.NA, end_ns=pd.NA,
                                     sample_count=pd.NA, sequence_id=pd.NA,
                                     duration_ns=pd.NA))
                continue
            for rec in df.to_dict("records"):
                all_rows.append({**base, **rec})

    return pd.DataFrame(all_rows)


# ---------------------------------------------------------------------------
# Sweep index parser
# ---------------------------------------------------------------------------

def load_sweep_index(out_dir: str | Path) -> pd.DataFrame:
    """Read the ``sweep_index.csv`` written by ``Sweep.run()``."""
    return pd.read_csv(Path(out_dir) / "sweep_index.csv")
