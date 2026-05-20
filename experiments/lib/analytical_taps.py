"""Method-of-images path delays for the §4.2 channel-validation overlay.

Mirrors ``src/channel/geometric_scene.cpp::compute_paths`` for the
direct / surface / bottom paths used in Experiment 1. The C++ scene
math is the ground truth; this is a hand-rolled Python copy that the
exp1 driver unit-tests against a one-shot C++ reference output.

Delays returned are *excess delays over the direct path* (matches the
simulator's tap-delay semantics and the cross-correlation peak relative
to the direct-path peak).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class AnalyticalTap:
    name: str            # "direct" | "surface" | "bottom"
    length_m: float
    excess_delay_s: float
    excess_delay_samples: float    # = excess_delay_s * fs_hz


def path_lengths(range_m: float,
                 *,
                 water_depth_m: float,
                 source_depth_m: float,
                 receiver_depth_m: float,
                 ) -> dict[str, float]:
    """Slant-range path lengths for direct / surface / bottom rays.

    Eq. (1) image-method form:
        ℓ_direct  = √(R² + (z_s − z_r)²)
        ℓ_surface = √(R² + (z_s + z_r)²)
        ℓ_bottom  = √(R² + ((D − z_s) + (D − z_r))²)
    """
    R = float(range_m)
    zs, zr, D = float(source_depth_m), float(receiver_depth_m), float(water_depth_m)
    dz_direct  = zs - zr
    dz_surface = zs + zr
    dz_bottom  = (D - zs) + (D - zr)
    return {
        "direct":  _hypot(R, dz_direct),
        "surface": _hypot(R, dz_surface),
        "bottom":  _hypot(R, dz_bottom),
    }


def analytical_taps(range_m: float,
                    *,
                    water_depth_m: float,
                    source_depth_m: float,
                    receiver_depth_m: float,
                    sound_speed_m_s: float = 1500.0,
                    fs_hz: float = 500_000.0,
                    paths: Sequence[str] = ("direct", "surface", "bottom"),
                    ) -> list[AnalyticalTap]:
    """Excess-over-direct delays (and sample counts) for the requested paths.

    Returned in *increasing-delay* order, which for the default geometry
    happens to be direct -> bottom -> surface but can flip in other
    configurations — callers should not assume a fixed name order.
    """
    lengths = path_lengths(range_m,
                           water_depth_m   = water_depth_m,
                           source_depth_m  = source_depth_m,
                           receiver_depth_m= receiver_depth_m)
    c = float(sound_speed_m_s)
    if c <= 0.0:
        raise ValueError("sound_speed_m_s must be > 0")
    direct_len = lengths["direct"]
    out: list[AnalyticalTap] = []
    for name in paths:
        if name not in lengths:
            raise KeyError(f"unknown path '{name}'; "
                           f"expected one of {sorted(lengths)}")
        L = lengths[name]
        excess_s = max(0.0, (L - direct_len) / c)
        out.append(AnalyticalTap(
            name                 = name,
            length_m             = L,
            excess_delay_s       = excess_s,
            excess_delay_samples = excess_s * float(fs_hz),
        ))
    out.sort(key=lambda t: t.excess_delay_samples)
    return out


def _hypot(a: float, b: float) -> float:
    # Avoid numpy import for a function this hot in tests.
    from math import sqrt
    return sqrt(a * a + b * b)
