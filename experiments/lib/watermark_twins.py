"""Geometric-twin parameters for the Watermark datasets.

For the replay-vs-geometric comparison, each dataset gets a "twin":
a geometric scenario configured from the deployment facts documented in
the Watermark paper (Table I) so both channel models describe the same
ocean. Range and water depth are published; source/receiver depths and
bottom properties are NOT in the paper table — the values here are
assumptions to refine from the FFI report / KAM11 trial docs before any
paper-grade run (each entry lists its assumptions).

Usage:

    from experiments.lib import watermark_twins as twins
    params = twins.twin_params("NOF1", seed=0)
    render_to_file(twins.TEMPLATE, params, out_dir / "scenario.yaml")
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

TEMPLATE = (Path(__file__).resolve().parents[1]
            / "configs" / "twin" / "geometric_twin.yaml.j2")


@dataclass(frozen=True)
class TwinGeometry:
    range_m: float             # published (Watermark Table I)
    water_depth_m: float       # published
    fc_meas_hz: float          # dataset probe centre frequency
    source_depth_m: float      # ASSUMED unless noted
    receiver_depth_m: float    # ASSUMED unless noted
    tow_speed_m_s: float       # 0 = fixed-fixed; KAU: tow ship, VERIFY speed
    gamma_bottom: float        # ASSUMED bottom reflection coefficient
    assumptions: str           # what needs verifying before a paper run


WATERMARK_TWINS: dict[str, TwinGeometry] = {
    # Oslofjord, fixed-fixed, 10-18 kHz.
    "NOF1": TwinGeometry(
        range_m=750.0, water_depth_m=10.0, fc_meas_hz=14_000.0,
        source_depth_m=4.0, receiver_depth_m=8.0,
        tow_speed_m_s=0.0, gamma_bottom=0.5,
        assumptions="source/receiver depths and bottom gamma assumed; "
                    "verify from FFI-rapport 16/01378"),
    # Norwegian shelf, fixed-fixed, 10-18 kHz.
    "NCS1": TwinGeometry(
        range_m=540.0, water_depth_m=80.0, fc_meas_hz=14_000.0,
        source_depth_m=30.0, receiver_depth_m=60.0,
        tow_speed_m_s=0.0, gamma_bottom=0.5,
        assumptions="source/receiver depths and bottom gamma assumed; "
                    "verify from FFI-rapport 16/01378"),
    # Brest harbour, fixed-fixed, 32.5-37.5 kHz (closest band to the
    # OpenAquatix modems).
    "BCH1": TwinGeometry(
        # Documented shallow deployment: source 2 m, receiver 3 m in 20 m water
        # at 800 m. At this geometry the direct and surface arrivals are only
        # ~10 us apart, so the modem's FH band is essentially flat: the 5-ray
        # model predicts near-error-free links and UNDER-predicts the recorded
        # BER, whose fading comes from diffuse reverberation the ray model omits.
        range_m=800.0, water_depth_m=20.0, fc_meas_hz=35_000.0,
        source_depth_m=2.0, receiver_depth_m=3.0,
        tow_speed_m_s=0.0, gamma_bottom=0.4,
        assumptions="source/receiver depths per Watermark (2 m / 3 m); bottom "
                    "gamma assumed; verify from FFI-rapport 16/01378"),
    # Kauai shelf (KAM11), towed source, 4-8 kHz. The tow gives real
    # platform motion — the bridge case against geometric R(t).
    "KAU1": TwinGeometry(
        range_m=1080.0, water_depth_m=100.0, fc_meas_hz=6_000.0,
        source_depth_m=50.0, receiver_depth_m=50.0,
        tow_speed_m_s=1.5, gamma_bottom=0.4,
        assumptions="tow speed, tow direction (sign of radial velocity), "
                    "source/receiver depths and bottom gamma assumed; "
                    "verify from the KAM11 trial documentation"),
    "KAU2": TwinGeometry(
        range_m=3160.0, water_depth_m=100.0, fc_meas_hz=6_000.0,
        source_depth_m=50.0, receiver_depth_m=50.0,
        tow_speed_m_s=1.5, gamma_bottom=0.4,
        assumptions="tow speed, tow direction, depths and bottom gamma "
                    "assumed; verify from the KAM11 trial documentation"),
}


def twin_params(dataset: str,
                *,
                seed: int = 0,
                run_duration_s: float = 60.0,
                channel_gain_db: float = -50.0,
                closing: bool = True,
                radial_velocity_override_m_s: float | None = None,
                modem_a_serial: str = "OA-2-1",
                modem_b_serial: str = "OA-2-2",
                ) -> dict:
    """Template parameter dict for one dataset's geometric twin.

    ``closing`` selects the sign of the tow motion (negative radial
    velocity = range decreasing). ``radial_velocity_override_m_s``
    replaces the documented tow speed with a signed measured value —
    pass the snapshot ensemble manifest's ``mean_v0_m_s`` so the twin
    moves at the dataset's measured mean Doppler (KAU1 files report
    ~-1.14 m/s against the 1.5 m/s documentation guess). The R envelope
    brackets the motion over ``run_duration_s`` plus a 10 % margin.
    """
    g = WATERMARK_TWINS[dataset]
    if radial_velocity_override_m_s is not None:
        v = float(radial_velocity_override_m_s)
    else:
        v = -abs(g.tow_speed_m_s) if closing else abs(g.tow_speed_m_s)
    swing = abs(v) * run_duration_s
    margin = max(0.1 * g.range_m, swing * 0.1 + 1.0)
    r_min = max(1.0, g.range_m - swing - margin)
    r_max = g.range_m + swing + margin
    return dict(
        dataset          = dataset,
        seed             = seed,
        range_m          = g.range_m,
        water_depth_m    = g.water_depth_m,
        source_depth_m   = g.source_depth_m,
        receiver_depth_m = g.receiver_depth_m,
        gamma_bottom     = g.gamma_bottom,
        velocity_radial_m_s = v,
        r_min_m          = r_min,
        r_max_m          = r_max,
        channel_gain_db  = channel_gain_db,
        modem_a_serial   = modem_a_serial,
        modem_b_serial   = modem_b_serial,
    )
