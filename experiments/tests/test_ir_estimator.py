"""Unit tests for the cross-correlation IR estimator.

Two synthetic tests per the Session F spec:

* Impulse-train TX with three known delayed copies — ``estimate_ir``
  recovers the three peaks within ±0 samples.
* Random chirp TX convolved with a known 3-tap IR — the recovered peaks
  match the IR taps within ±1 sample.
"""
from __future__ import annotations

import numpy as np
import pytest

from experiments.lib.ir_estimator import (
    estimate_ir_arrays,
    find_peaks,
)


FS = 500_000.0


def _random_broadband(length: int, seed: int = 0) -> np.ndarray:
    """A Gaussian random sequence — its autocorrelation is approximately a
    delta, so convolving it with a sparse IR and cross-correlating against
    the original recovers the IR taps cleanly. Replaces the spec's
    "impulse train" because periodic trains create ambiguous correlation
    peaks at multiples of the period."""
    rng = np.random.default_rng(seed)
    return rng.standard_normal(length)


def _lfm_chirp(length: int, f0: float, f1: float, fs: float) -> np.ndarray:
    """Hand-rolled linear chirp; phase = 2π·(f0·t + 0.5·k·t²)."""
    t = np.arange(length) / fs
    k = (f1 - f0) / (length / fs)
    phase = 2.0 * np.pi * (f0 * t + 0.5 * k * t * t)
    return np.cos(phase)


def _make_rx_with_taps(tx: np.ndarray,
                      taps_samples_amp: list[tuple[int, float]],
                      total_length: int | None = None) -> np.ndarray:
    """Convolve ``tx`` with a sparse 3-tap IR by summing scaled+shifted copies."""
    if total_length is None:
        total_length = tx.size + max(d for d, _ in taps_samples_amp) + 100
    rx = np.zeros(total_length, dtype=np.float64)
    for delay, amp in taps_samples_amp:
        rx[delay:delay + tx.size] += amp * tx
    return rx


# ---------------------------------------------------------------------------
# Impulse-train test — exact peak recovery
# ---------------------------------------------------------------------------

def test_broadband_signal_recovers_known_delays_exactly() -> None:
    """TX is a broadband (Gaussian-noise) signal with near-delta
    autocorrelation; RX is three delayed copies. Cross-correlation
    recovers all three peak locations within ±0 samples."""
    tx = _random_broadband(length=4096, seed=0)
    delays = [(0, 1.0), (200, 0.7), (1500, 0.4)]
    rx = _make_rx_with_taps(tx, delays, total_length=8192)

    delay_samples, mag = estimate_ir_arrays(
        tx, rx, fs_hz=FS, max_delay_s=0.005)
    peaks = find_peaks(delay_samples, mag,
                       n_peaks=3, min_separation_samples=50,
                       threshold_rel=0.1)
    recovered = sorted(int(round(d)) for d, _ in peaks)
    assert recovered == [0, 200, 1500], recovered


# ---------------------------------------------------------------------------
# Chirp + 3-tap IR — tap recovery within ±1 sample
# ---------------------------------------------------------------------------

def test_chirp_with_three_tap_ir_recovers_within_one_sample() -> None:
    rng = np.random.default_rng(42)
    chirp = _lfm_chirp(length=4096, f0=20_000.0, f1=30_000.0, fs=FS)
    # Make the TX deterministic but not a pure tone — sum chirp + a tiny
    # bit of noise so the autocorrelation is well-localised.
    tx = chirp + 0.01 * rng.standard_normal(chirp.size)

    delays = [(0, 1.0), (350, -0.55), (1200, 0.35)]
    rx = _make_rx_with_taps(tx, delays, total_length=8192)

    delay_samples, mag = estimate_ir_arrays(
        tx, rx, fs_hz=FS, max_delay_s=0.005)
    peaks = find_peaks(delay_samples, mag,
                       n_peaks=3, min_separation_samples=50,
                       threshold_rel=0.1)
    recovered = sorted(int(round(d)) for d, _ in peaks)
    expected = sorted(d for d, _ in delays)
    for r, e in zip(recovered, expected):
        assert abs(r - e) <= 1, f"peak {r} vs expected {e}"


def test_zero_length_inputs_raise() -> None:
    with pytest.raises(ValueError):
        estimate_ir_arrays(np.array([]), np.zeros(10))
    with pytest.raises(ValueError):
        estimate_ir_arrays(np.zeros(10), np.array([]))


def test_normalisation_makes_peak_one() -> None:
    tx = _random_broadband(length=2048, seed=1)
    rx = _make_rx_with_taps(tx, [(0, 5.0)], total_length=4096)
    _, mag = estimate_ir_arrays(tx, rx, fs_hz=FS, max_delay_s=0.001)
    assert mag.max() == pytest.approx(1.0)
    assert mag.min() >= 0.0


def test_recovers_taps_when_direct_path_is_far_from_lag_zero() -> None:
    """The real Exp 1 case: RX has hundreds of milliseconds of leading
    silence (or pre-chirp signal) before the channel-processed chirp
    appears. The IR estimator must search beyond the multipath window
    for the direct-path peak; if it doesn't, every tap measurement
    saturates at the search window edge."""
    tx = _random_broadband(length=4096, seed=4)
    # Direct path at lag 100_000, surface at +200, bottom at +1500 (matching
    # exp1's R≈500m one-way time of flight measured in samples at 500 kSPS).
    delays = [(100_000, 1.0), (100_200, 0.7), (101_500, 0.4)]
    rx = _make_rx_with_taps(tx, delays, total_length=200_000)

    delay_samples, mag = estimate_ir_arrays(
        tx, rx, fs_hz=FS, max_delay_s=0.005)
    # `delay_samples` is excess-over-direct, so peaks should be at 0, 200, 1500.
    peaks = find_peaks(delay_samples, mag,
                       n_peaks=3, min_separation_samples=50,
                       threshold_rel=0.1)
    recovered = sorted(int(round(d)) for d, _ in peaks)
    assert recovered == [0, 200, 1500], recovered


def test_window_truncates_to_requested_max_delay() -> None:
    tx = _random_broadband(length=1024, seed=2)
    rx = _make_rx_with_taps(tx, [(0, 1.0), (5000, 0.5)],
                            total_length=8192)
    delay_samples, mag = estimate_ir_arrays(
        tx, rx, fs_hz=FS, max_delay_s=0.001)    # 500 samples
    assert mag.size == 501                       # inclusive of lag=0..500
    # The peak at +5000 samples is outside the window, so the only peak
    # in mag should be the lag-0 direct peak.
    assert int(np.argmax(mag)) == 0
