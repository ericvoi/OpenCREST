"""Determinism check.

Run the same scenario twice and verify every output artifact is bit-equal
under a per-artifact normalisation rule. Used as a pre-flight before any
expensive sweep -- if the simulator drifted into nondeterminism (race
condition in init, time-dependent default, etc.) the cross-seed sweeps
that follow would not be trustworthy.

Normalisation rules:

* **WAV**: SHA256 over the normalised PCM stream (float64, mono coercion).
  Header fields (sizes, lengths) are excluded because the streaming writer
  may leave them at zero on signal-driven shutdown.
* **CDC log**: SHA256 over ``<line_text>`` only, stripping the host
  timestamp prefix.
* **Event JSONL**: SHA256 over ``<modem_id, direction, sequence_id, sample_count>``
  per line; the wall-clock ``start_ns``/``end_ns`` fields are
  excluded because they're real-time-dependent.
* **Summary JSON**: SHA256 over a documented subset of fields:
  ``random_seed``, ``processing_time.{p50_us, p95_us, p99_us, count, underrun_count}``,
  ``channel_engine.*``. Excludes timestamps and the ``processing_time.mean_us``
  rounding-flutter field.
"""
from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import metrics_loader as ml
from . import wav_io


_CDC_PREFIX_RE = re.compile(r"^\[\d+ ns\] ")


# ---------------------------------------------------------------------------
# Fingerprint helpers
# ---------------------------------------------------------------------------

def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def fingerprint_wav(path: str | Path) -> str:
    wav = wav_io.read_wav(path)
    arr = np.ascontiguousarray(wav.samples, dtype=np.float64)
    return _sha256(arr.tobytes())


def fingerprint_cdc(path: str | Path) -> str:
    lines = []
    for raw in Path(path).read_text().splitlines():
        lines.append(_CDC_PREFIX_RE.sub("", raw))
    return _sha256("\n".join(lines).encode("utf-8"))


def fingerprint_events(path: str | Path) -> str:
    rows: list[str] = []
    with Path(path).open() as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            rows.append("|".join([
                str(obj.get("modem_id", "")),
                str(obj.get("direction", "")),
                str(obj.get("sequence_id", "")),
                str(obj.get("sample_count", "")),
            ]))
    return _sha256("\n".join(rows).encode("utf-8"))


_SUMMARY_DETERMINISTIC_FIELDS = (
    ("random_seed",),
    ("processing_time", "count"),
    ("processing_time", "p50_us"),
    ("processing_time", "p95_us"),
    ("processing_time", "p99_us"),
    ("processing_time", "max_us"),
    ("processing_time", "underrun_count"),
    ("channel_engine", "rx_ring_underruns"),
    ("channel_engine", "fw_rx_underruns"),
    ("channel_engine", "tx_packets_total"),
    ("channel_engine", "rx_packets_total"),
)


def fingerprint_summary(path: str | Path) -> str:
    doc = ml.load_summary(path)
    subset = {}
    for fld in _SUMMARY_DETERMINISTIC_FIELDS:
        node: object = doc
        for key in fld:
            if not isinstance(node, dict) or key not in node:
                node = None
                break
            node = node[key]
        subset[".".join(fld)] = node
    canonical = json.dumps(subset, sort_keys=True,
                           separators=(",", ":")).encode("utf-8")
    return _sha256(canonical)


# ---------------------------------------------------------------------------
# Whole-cell diff
# ---------------------------------------------------------------------------

@dataclass
class ArtifactResult:
    artifact: str
    fingerprint_a: str
    fingerprint_b: str

    @property
    def equal(self) -> bool:
        return self.fingerprint_a == self.fingerprint_b


@dataclass
class DeterminismReport:
    results: list[ArtifactResult] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.results) and all(r.equal for r in self.results)

    def to_text(self) -> str:
        if not self.results:
            return "no artifacts compared"
        width = max(len(r.artifact) for r in self.results)
        out: list[str] = []
        for r in self.results:
            mark = "PASS" if r.equal else "FAIL"
            out.append(f"  [{mark}] {r.artifact:<{width}} "
                       f"a={r.fingerprint_a[:12]}  b={r.fingerprint_b[:12]}")
        header = f"determinism: {'OK' if self.ok else 'MISMATCH'} "\
                 f"({sum(r.equal for r in self.results)}/{len(self.results)} match)"
        return "\n".join([header, *out])


def compare_cells(cell_a: str | Path, cell_b: str | Path) -> DeterminismReport:
    """Compare every paired artifact in two cell directories."""
    A = Path(cell_a)
    B = Path(cell_b)
    results: list[ArtifactResult] = []

    def add(label: str, fn, pa: Path, pb: Path) -> None:
        if not pa.is_file() or not pb.is_file():
            return
        results.append(ArtifactResult(label, fn(pa), fn(pb)))

    for wav in sorted(A.glob("*.wav")):
        add(f"wav:{wav.name}",     fingerprint_wav,     wav, B / wav.name)
    for cdc in sorted(A.glob("*_cdc.log")):
        add(f"cdc:{cdc.name}",     fingerprint_cdc,     cdc, B / cdc.name)
    for ev in sorted(A.glob("*_events.jsonl")):
        add(f"events:{ev.name}",   fingerprint_events,  ev, B / ev.name)
    for js in sorted(A.glob("*_summary.json")):
        add(f"summary:{js.name}",  fingerprint_summary, js, B / js.name)

    return DeterminismReport(results=results)
