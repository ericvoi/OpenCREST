#!/usr/bin/env python3
"""Stub openCREST binary used by the runner smoke test.

Mimics enough of the simulator's contract that the Python ``Sweep``
orchestrator's process-management and result-collection paths can be
exercised end-to-end without USB hardware or the C++ binary.

The stub:

* Parses ``<scenario.yaml>`` to learn ``logging.output_directory`` and
  ``name``.
* Writes:
    - ``<output_dir>/<name>_summary.json``
    - ``<output_dir>/<modem_id>_tx.wav``      (minimal RIFF, 1 sample)
    - ``<output_dir>/<modem_id>_events.jsonl`` (a few synthetic events)
* Sleeps until SIGTERM or ``--max-runtime-s`` (default 60 s), whichever
  comes first; SIGTERM produces a clean exit code 0.

Output is byte-deterministic for a given scenario YAML so the
determinism check can fingerprint repeat runs.
"""
from __future__ import annotations

import argparse
import json
import signal
import struct
import sys
import time
from pathlib import Path

import yaml


_stop = False


def _on_signal(signum, frame):                                  # noqa: ARG001
    global _stop
    _stop = True


def _write_minimal_wav(path: Path) -> None:
    """Write a 1-sample mono WAV at 500 kSPS. Deterministic content."""
    sample_rate = 500_000
    nchannels   = 1
    sampwidth   = 2     # bytes per sample
    samples     = b"\x00\x00"

    data_size = len(samples)
    fmt_chunk = struct.pack(
        "<4sIHHIIHH",
        b"fmt ", 16,
        1,                  # PCM format
        nchannels,
        sample_rate,
        sample_rate * nchannels * sampwidth,
        nchannels * sampwidth,
        sampwidth * 8,
    )
    data_chunk = struct.pack("<4sI", b"data", data_size) + samples
    riff_size  = 4 + len(fmt_chunk) + len(data_chunk)
    riff       = struct.pack("<4sI4s", b"RIFF", riff_size, b"WAVE")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(riff + fmt_chunk + data_chunk)


def _write_events_jsonl(path: Path, modem_id: str) -> None:
    """Write a deterministic 3-event JSONL stream."""
    rows = [
        dict(modem_id=modem_id, direction="Tx", start_ns=1_000_000,
             end_ns=2_000_000,    sample_count=512, sequence_id=0),
        dict(modem_id=modem_id, direction="Rx", start_ns=3_000_000,
             end_ns=4_500_000,    sample_count=768, sequence_id=1),
        dict(modem_id=modem_id, direction="Tx", start_ns=6_000_000,
             end_ns=7_200_000,    sample_count=600, sequence_id=2),
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as fp:
        for r in rows:
            fp.write(json.dumps(r, sort_keys=True) + "\n")


def _write_summary(path: Path, scenario: dict, scenario_path: Path) -> None:
    name = scenario.get("name", "run")
    seed = int(scenario.get("random_seed", 0))
    modems = [str(m["id"]) for m in scenario.get("modems", [])]

    payload = {
        "scenario_name":  name,
        "scenario_path":  str(scenario_path),
        "random_seed":    seed,
        # Fixed timestamp strings; the determinism fingerprint excludes
        # them anyway.
        "started_at":     "1970-01-01T00:00:00Z",
        "ended_at":       "1970-01-01T00:00:01Z",
        "duration_s":     1.0,
        "modems":         modems,
        "processing_time": {
            "count":          100,
            "mean_us":        200.0,
            "p50_us":         180,
            "p95_us":         310,
            "p99_us":         360,
            "max_us":         420,
            "underrun_count": 0,
        },
        "channel_engine": {
            "rx_ring_underruns": 0,
            "fw_rx_underruns":   0,
            "tx_packets_total":  100,
            "rx_packets_total":  100,
        },
        "log_files": {
            "events": [f"{m}_events.jsonl" for m in modems],
            "cdc":    [],
            "wav":    [f"{m}_tx.wav" for m in modems],
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario_path", type=Path)
    ap.add_argument("--max-runtime-s", type=float, default=60.0,
                    help="cap on stub runtime; exits 0 on SIGTERM sooner")
    ap.add_argument("--ignore-sigterm", action="store_true",
                    help="install a no-op SIGTERM handler so the watchdog "
                         "path in Sweep (SIGKILL fallback after grace) "
                         "can be exercised")
    args = ap.parse_args()

    if args.ignore_sigterm:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    else:
        signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT,  _on_signal)

    scenario = yaml.safe_load(args.scenario_path.read_text())
    if not isinstance(scenario, dict):
        sys.stderr.write("stub: scenario YAML did not parse to a mapping\n")
        return 2

    out_dir_raw = scenario.get("logging", {}).get("output_directory", ".")
    out_dir = Path(out_dir_raw)
    if not out_dir.is_absolute():
        out_dir = (args.scenario_path.parent / out_dir).resolve()

    name = scenario.get("name", "run")
    modem_ids = [str(m["id"]) for m in scenario.get("modems", [])] or ["modem_a"]

    # Emit artifacts up-front so a fast SIGTERM still finds them in place.
    _write_summary(out_dir / f"{name}_summary.json", scenario, args.scenario_path)
    for mid in modem_ids:
        _write_minimal_wav(out_dir / f"{mid}_tx.wav")
        _write_events_jsonl(out_dir / f"{mid}_events.jsonl", mid)

    sys.stdout.write(f"stub: wrote artifacts under {out_dir}\n")
    sys.stdout.flush()

    deadline = time.monotonic() + args.max_runtime_s
    while not _stop and time.monotonic() < deadline:
        time.sleep(0.05)

    sys.stdout.write("stub: exiting cleanly\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
