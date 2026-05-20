"""Smoke tests for the analytical method-of-images path delays.

Cross-checks against:
* Hand-computed slant ranges for the paper geometry
  (D=120, z_s=50, z_r=100).
* The C++ ``GeometricScene`` unit tests in
  ``tests/unit/test_geometric_scene.cpp`` use identical formulas — these
  Python tests assert the same closed-form values without spawning a
  subprocess. The C++ side is the ground truth; if either drifts, the
  C++ tests will tell us first.
"""
from __future__ import annotations

import math

import pytest

from experiments.lib.analytical_taps import analytical_taps, path_lengths


GEOM = dict(water_depth_m=120.0, source_depth_m=50.0, receiver_depth_m=100.0)


def test_path_lengths_at_R_500() -> None:
    L = path_lengths(500.0, **GEOM)
    assert L["direct"]  == pytest.approx(math.sqrt(500**2 +  50**2), abs=1e-3)
    assert L["surface"] == pytest.approx(math.sqrt(500**2 + 150**2), abs=1e-3)
    assert L["bottom"]  == pytest.approx(math.sqrt(500**2 +  90**2), abs=1e-3)


def test_direct_excess_is_zero() -> None:
    for R in (200.0, 500.0, 1000.0):
        taps = analytical_taps(R, **GEOM)
        direct = next(t for t in taps if t.name == "direct")
        assert direct.excess_delay_s == 0.0
        assert direct.excess_delay_samples == 0.0


def test_surface_is_longer_path_than_bottom_for_paper_geometry() -> None:
    """With z_s=50, z_r=100, D=120: dz_surface = 150 > dz_bottom = 90, so
    the surface bounce always arrives after the bottom bounce."""
    for R in (200.0, 500.0, 1000.0):
        taps = analytical_taps(R, **GEOM)
        bottom = next(t for t in taps if t.name == "bottom").excess_delay_samples
        surface = next(t for t in taps if t.name == "surface").excess_delay_samples
        assert bottom < surface


def test_excess_delay_samples_match_handcomputed_R_500() -> None:
    taps = analytical_taps(500.0, fs_hz=500_000.0, **GEOM)
    by_name = {t.name: t for t in taps}

    expected_surface_samples = (math.sqrt(500**2 + 150**2)
                                 - math.sqrt(500**2 +  50**2)) / 1500.0 * 500_000.0
    expected_bottom_samples  = (math.sqrt(500**2 +  90**2)
                                 - math.sqrt(500**2 +  50**2)) / 1500.0 * 500_000.0

    assert by_name["surface"].excess_delay_samples == pytest.approx(
        expected_surface_samples, abs=1e-3)
    assert by_name["bottom"].excess_delay_samples == pytest.approx(
        expected_bottom_samples, abs=1e-3)


def test_ordering_is_by_increasing_delay() -> None:
    """``analytical_taps`` returns increasing-delay order regardless of
    the input path tuple."""
    for R in (200.0, 1000.0):
        taps = analytical_taps(R, paths=("surface", "direct", "bottom"), **GEOM)
        samples = [t.excess_delay_samples for t in taps]
        assert samples == sorted(samples)


def test_sound_speed_must_be_positive() -> None:
    with pytest.raises(ValueError):
        analytical_taps(500.0, sound_speed_m_s=0.0, **GEOM)


def test_unknown_path_name_raises() -> None:
    with pytest.raises(KeyError):
        analytical_taps(500.0, paths=("nope",), **GEOM)
