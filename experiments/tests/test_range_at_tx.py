"""Tests for exp3's range-at-TX computation.

The Session H plan specifies::

    R_tx = R_0 + v · t_tx_start

where ``t_tx_start = (event.start_ns - simulator_spawn_ns) / 1e9``. With
the project's "closing platform is negative velocity" convention, v=-2
shrinks R over time.
"""
from __future__ import annotations

import math

import pytest

from experiments.exp3_janus_per import compute_range_at_tx


def test_zero_elapsed_returns_r0() -> None:
    assert compute_range_at_tx(
        event_start_ns      = 1_000_000_000,
        simulator_spawn_ns  = 1_000_000_000,
        r0_m                = 1000.0,
        v_m_s               = -2.0,
    ) == pytest.approx(1000.0)


def test_closing_at_minus_2_after_100s() -> None:
    """v=-2 m/s, 100 s elapsed → R = 1000 + (-2)·100 = 800 m."""
    assert compute_range_at_tx(
        event_start_ns      = 1_000_000_000 + 100 * 1_000_000_000,
        simulator_spawn_ns  = 1_000_000_000,
        r0_m                = 1000.0,
        v_m_s               = -2.0,
    ) == pytest.approx(800.0)


def test_stationary_zero_velocity_is_constant() -> None:
    """v=0 → range never changes, regardless of elapsed time."""
    for t_s in (0.0, 1.0, 100.0, 500.0):
        assert compute_range_at_tx(
            event_start_ns      = int(t_s * 1e9),
            simulator_spawn_ns  = 0,
            r0_m                = 500.0,
            v_m_s               = 0.0,
        ) == pytest.approx(500.0)


def test_opening_positive_velocity() -> None:
    """v=+1 m/s, 50 s elapsed → R = 500 + 50 = 550 m."""
    assert compute_range_at_tx(
        event_start_ns      = 50_000_000_000,
        simulator_spawn_ns  = 0,
        r0_m                = 500.0,
        v_m_s               = +1.0,
    ) == pytest.approx(550.0)


def test_subsecond_resolution() -> None:
    """A 1-ms-resolution shift should produce a 2-mm range change at v=-2."""
    r_a = compute_range_at_tx(0,             0, 1000.0, -2.0)
    r_b = compute_range_at_tx(1_000_000,     0, 1000.0, -2.0)        # +1 ms
    assert (r_a - r_b) == pytest.approx(0.002, abs=1e-9)


def test_returns_finite_float() -> None:
    """Sanity: result is a float (not numpy / decimal / etc.), and finite."""
    r = compute_range_at_tx(123_456_789, 0, 1000.0, -2.0)
    assert isinstance(r, float)
    assert math.isfinite(r)
