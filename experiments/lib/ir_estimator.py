"""Cross-correlation impulse-response estimator for Experiment 1.

The Session-A LFM chirp is launched on modem A. Modem B's RX captures the
channel-modified copy. Cross-correlating ``rx`` against the logged ``tx``
recovers the channel's IR over the positive-lag window: the global peak is
the direct path, secondary peaks are surface / bottom reflections.

Using the *logged* TX rather than an idealized chirp replica removes the
transducer's electrical-domain shaping from the estimate — what we want is
the channel-only IR, and that's exactly ``H_channel = xcorr(rx, tx_logged) /
auto_corr(tx_logged)`` at lag = 0 (i.e. relative timing of the peaks).
For paper §4.2 we only care about the *peak locations*, so the
normalisation to a true deconvolution is unnecessary; a normalised xcorr
magnitude is sufficient.
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
    """Cross-correlate ``rx`` against ``tx`` and return the impulse
    response anchored at the direct-path peak.

    The TX WAV captures the chirp at (or near) the start of its TX
    session; the RX WAV captures the channel-processed copy at whatever
    delay corresponds to the chirp's propagation through the simulator
    (one-way time-of-flight + base-delay) — that's hundreds of
    milliseconds even at R=200m, far outside the multipath window. So
    we first scan ``[0, search_window_s]`` of positive lags for the
    global magnitude peak — that's the direct path — then return the
    ``max_delay_s``-wide window starting *at* that peak.

    ``delay_samples[0] = 0`` represents the direct-path arrival; later
    indices are excess delays in samples (matches
    ``analytical_taps``' semantics). ``mag`` is normalised to peak 1.0.

    Both WAVs must be at the same sample rate (the simulator writes
    both at ``modems[*].sample_rate``); ``fs_hz`` is used only for
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

    # Search for the direct-path peak. The available positive-lag range is
    # rx.size - 1 samples; clamp search_window_s to that.
    max_positive_lag = full.size - 1 - zero_idx
    search_max_lag = min(int(round(search_window_s * fs_hz)),
                          max_positive_lag)
    if search_max_lag <= 0:
        raise ValueError(
            "estimate_ir_arrays: rx must be longer than tx for a positive-"
            "lag search")
    search_window = np.abs(full[zero_idx : zero_idx + search_max_lag + 1])
    direct_lag = int(np.argmax(search_window))

    # Return the IR window starting at direct_lag.
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
    """Pick up to ``n_peaks`` local maxima in ``mag`` for tap-delay
    extraction.

    Local maxima are identified with a strict-greater test against
    immediate neighbours, then greedily selected in descending magnitude
    while enforcing a minimum sample-distance from already-chosen peaks.
    Endpoint samples qualify as maxima (the direct path sits at sample
    0).

    Returns a list of ``(delay_samples, magnitude)`` tuples sorted by
    increasing delay. Items below ``threshold_rel * peak`` are dropped.
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
        # Degenerate input (e.g. monotonic); fall back to global argmax.
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
