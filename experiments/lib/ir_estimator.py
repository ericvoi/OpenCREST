"""Cross-correlation impulse-response estimator for Experiment 1.

A firmware LFM chirp is launched on modem A; modem B's RX captures the
channel-modified copy. Cross-correlating ``rx`` against the logged ``tx``
recovers the channel IR over the positive-lag window: the global peak is
the direct path, secondary peaks are surface / bottom reflections.

Using the *logged* TX rather than an idealized chirp replica removes the
transducer's electrical-domain shaping from the estimate. Only peak
locations are needed downstream, so a normalised xcorr magnitude
suffices in place of a true deconvolution.
"""
from __future__ import annotations

from pathlib import Path
from typing import Tuple

import numpy as np
from scipy.signal import correlate

from . import wav_io


def estimate_ir(tx_wav: str | Path,
                rx_wav: str | Path,
                *,
                fs_hz: float = 500_000.0,
                max_delay_s: float = 0.035,
                search_window_s: float = 2.0,
                ) -> Tuple[np.ndarray, np.ndarray]:
    """Cross-correlate ``rx`` against ``tx`` and return the impulse response
    anchored at the direct-path peak.

    Because the chirp arrives at the RX after the full propagation delay
    (hundreds of milliseconds even at short ranges), we first scan
    ``[0, search_window_s]`` of positive lags for the global magnitude peak
    (the direct path) and then return the ``max_delay_s``-wide window
    starting at that peak. ``delay_samples[0] = 0`` is the direct-path
    arrival; later indices are excess delays in samples (matches
    ``analytical_taps``). ``mag`` is normalised to peak 1.0.

    Both WAVs must be at the same sample rate; ``fs_hz`` is used only for
    converting times to samples.
    """
    tx_samples = wav_io.read_wav(tx_wav).samples
    rx_samples = wav_io.read_wav(rx_wav).samples
    tx = _to_mono_float(tx_samples)
    rx = _to_mono_float(rx_samples)
    return estimate_ir_arrays(tx, rx, fs_hz=fs_hz, max_delay_s=max_delay_s,
                              search_window_s=search_window_s)


def estimate_ir_arrays(tx: np.ndarray,
                       rx: np.ndarray,
                       *,
                       fs_hz: float = 500_000.0,
                       max_delay_s: float = 0.035,
                       search_window_s: float = 2.0,
                       ) -> Tuple[np.ndarray, np.ndarray]:
    """Array-domain entry point (testable without WAV files)."""
    tx = np.asarray(tx, dtype=np.float64).reshape(-1)
    rx = np.asarray(rx, dtype=np.float64).reshape(-1)
    if tx.size == 0 or rx.size == 0:
        raise ValueError("estimate_ir_arrays: tx and rx must be non-empty")

    # scipy.signal.correlate(rx, tx, 'full') puts lag=0 at index tx.size-1;
    # positive lags = rx arrives later than tx by `lag` samples.
    full = correlate(rx, tx, mode="full", method="auto")
    zero_idx = tx.size - 1

    max_positive_lag = full.size - 1 - zero_idx
    search_max_lag = min(int(round(search_window_s * fs_hz)),
                          max_positive_lag)
    if search_max_lag <= 0:
        raise ValueError(
            "estimate_ir_arrays: rx must be longer than tx for a positive-"
            "lag search")
    search_window = np.abs(full[zero_idx : zero_idx + search_max_lag + 1])
    direct_lag = int(np.argmax(search_window))

    window_len = int(round(max_delay_s * fs_hz)) + 1
    start_idx = zero_idx + direct_lag
    end_idx = min(start_idx + window_len - 1, full.size - 1)
    out_window = full[start_idx : end_idx + 1]
    mag = np.abs(out_window)
    peak = float(mag.max())
    if peak > 0.0:
        mag = mag / peak
    delay_samples = np.arange(mag.size, dtype=np.float64)
    return delay_samples, mag


def find_peaks(delay_samples: np.ndarray,
               mag: np.ndarray,
               *,
               n_peaks: int = 3,
               min_separation_samples: int = 4,
               threshold_rel: float = 0.05,
               ) -> list[tuple[float, float]]:
    """Pick up to ``n_peaks`` local maxima in ``mag`` for tap-delay extraction.

    Maxima are identified with a strict-greater test against immediate
    neighbours (endpoint samples qualify, so the direct path at sample 0
    is included), then greedily selected in descending magnitude while
    enforcing a minimum sample-distance from already-chosen peaks. Items
    below ``threshold_rel * peak`` are dropped. Result is sorted by
    increasing delay.
    """
    if delay_samples.shape != mag.shape:
        raise ValueError("delay_samples and mag must have matching shape")
    if mag.size == 0:
        return []

    n = mag.size
    candidates: list[int] = []
    for i in range(n):
        left  = mag[i - 1] if i > 0     else -np.inf
        right = mag[i + 1] if i < n - 1 else -np.inf
        if mag[i] > left and mag[i] >= right:
            candidates.append(i)
    if not candidates:
        # Monotonic / degenerate input: fall back to global argmax.
        candidates = [int(np.argmax(mag))]

    threshold = threshold_rel * float(mag.max())
    candidates.sort(key=lambda i: mag[i], reverse=True)

    chosen: list[int] = []
    for idx in candidates:
        if mag[idx] < threshold:
            break
        if any(abs(idx - j) < min_separation_samples for j in chosen):
            continue
        chosen.append(idx)
        if len(chosen) >= n_peaks:
            break

    chosen.sort()
    return [(float(delay_samples[i]), float(mag[i])) for i in chosen]


def _to_mono_float(samples: np.ndarray) -> np.ndarray:
    arr = np.asarray(samples, dtype=np.float64)
    if arr.ndim == 2:
        arr = arr.mean(axis=1)
    return arr
