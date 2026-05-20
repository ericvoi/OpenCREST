"""WAV read helpers.

The openCREST streaming writer leaves RIFF/data chunk sizes at 0 if it
didn't close gracefully (e.g. SIGTERM / Ctrl-C). ``scripts/analyze_wav.py``
already has a tolerant parser; this module re-implements the same logic
here so the experiment harness doesn't have to import from ``scripts/``.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class WavSamples:
    rate: int
    sampwidth: int    # bytes
    nchannels: int
    samples: np.ndarray   # shape (n,) for mono, (n, nchannels) for stereo


def read_wav(path: str | Path) -> WavSamples:
    """Read ``path`` and return a :class:`WavSamples` with mono samples
    coerced to ``float64`` in the range -1..+1.

    Tolerant of headers where RIFF / data chunk sizes are 0 (interrupted
    writer): falls back to reading until EOF.
    """
    p = Path(path)
    with p.open("rb") as f:
        if f.read(4) != b"RIFF":
            raise ValueError(f"{p}: not a RIFF file")
        f.read(4)                                  # riff size ignored
        if f.read(4) != b"WAVE":
            raise ValueError(f"{p}: not a WAVE file")
        rate = sampwidth = nchannels = None
        data_offset = data_size = None
        while True:
            hdr = f.read(8)
            if len(hdr) < 8:
                break
            chunk_id, chunk_size = struct.unpack("<4sI", hdr)
            if chunk_id == b"fmt ":
                fmt = f.read(chunk_size)
                _, nchannels, rate, _, _, bits = struct.unpack(
                    "<HHIIHH", fmt[:16])
                sampwidth = bits // 8
            elif chunk_id == b"data":
                data_offset = f.tell()
                data_size = chunk_size
                break
            else:
                f.seek(chunk_size, 1)
        if data_offset is None or rate is None or sampwidth is None:
            raise ValueError(f"{p}: missing fmt or data chunk")
        f.seek(0, 2)
        eof = f.tell()
        f.seek(data_offset)
        end = eof if (data_size == 0
                      or data_offset + data_size > eof) else \
              data_offset + data_size
        raw = f.read(end - data_offset)

    if sampwidth == 2:
        arr = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    elif sampwidth == 1:
        arr = (np.frombuffer(raw, dtype=np.uint8).astype(np.float64) - 128.0) / 128.0
    elif sampwidth == 4:
        arr = np.frombuffer(raw, dtype="<i4").astype(np.float64) / (2 ** 31)
    else:
        raise ValueError(f"{p}: unsupported sample width {sampwidth}")

    if nchannels > 1:
        arr = arr.reshape(-1, nchannels)

    return WavSamples(rate=int(rate), sampwidth=int(sampwidth),
                      nchannels=int(nchannels), samples=arr)
