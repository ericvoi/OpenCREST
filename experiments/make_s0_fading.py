"""S0-v2: does a DEEP spectral notch (and its Doppler sweep) reproduce
the BER floor that a single shallow -6 dB echo did not?

The first S0 pass showed a -6 dB static echo cleans to ~0 BER by high
SNR. Two things make real multipath worse, and both need
*comparable-amplitude* taps to bite:

  * snap0/1  EQUAL static  — two 0.7 taps -> a near-total spectral NULL
    at a fixed frequency. Tests whether a deep static notch alone floors
    (the 16-tap real channel makes deep notches from tap richness).
  * snap2/3  EQUAL + Doppler — same two taps, but the echo delay RAMPS
    (relative Doppler), so the notch SWEEPS across the tone set over the
    message. Tests time-selective fading, which no average SNR fixes.

Doppler is encoded as a linear delay ramp: dtau/dt = -nu/fc, the same
convention the quasi-static converter uses, so the existing replay
renderer plays it back unchanged.

Reading the sweep:
  * equal-static floors                 -> deep spectral notch (need EQ).
  * equal-static clean, Doppler floors  -> time-selective fading is the
    driver (need faster re-sync / interleave+code, not EQ).
  * both floor                          -> both contribute.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from experiments.lib.tap_trajectory import read_octt, write_octt

REPO = Path(__file__).resolve().parent.parent
TEMPLATE_OCTT = REPO / "experiments/results/ensembles/BCH1/BCH1_snap00.octt"
OUT_DIR = REPO / "experiments/results/ensembles/BCH1_fading"

DIRECT_DELAY_S = 2e-5
ECHO_DELAY_S = 2e-3           # 50% of the 4 ms symbol; notch every 500 Hz
EQUAL_AMP = 0.7071           # equal taps -> deep null (|a-a| = 0)
# Two fade rates (upward ramps only, so echo delay stays positive over the
# whole record). Different relative Doppler = different notch sweep speed.
REL_DOPPLER_HZ = (5.0, 9.0)


def _write(out_dir: Path, name: str, grid, echo_ramp_rate_s_per_s: float,
           ) -> dict:
    n = grid.frame_count
    t = np.arange(n) * grid.dt_s               # record time per frame

    delays = np.empty((n, 2), dtype=np.float64)
    delays[:, 0] = DIRECT_DELAY_S              # direct path, static
    # Echo: constant excess + optional linear ramp (Doppler). Ramp kept
    # gentle so the delay stays well inside [guard, max_delay] over 30 s.
    delays[:, 1] = DIRECT_DELAY_S + ECHO_DELAY_S + echo_ramp_rate_s_per_s * t

    amps = np.full((n, 2), EQUAL_AMP, dtype=np.float32)

    path = out_dir / f"{name}.octt"
    write_octt(path, grid.dt_s, grid.fc_meas_hz, delays, amps)
    return {
        "octt_file": f"{name}.octt",
        "kind": name,
        "echo_delay_s": ECHO_DELAY_S,
        "echo_ramp_rate_s_per_s": echo_ramp_rate_s_per_s,
        "rel_doppler_hz": -echo_ramp_rate_s_per_s * grid.fc_meas_hz,
        "amp_each": EQUAL_AMP,
    }


def main() -> int:
    grid = read_octt(TEMPLATE_OCTT)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    r5 = REL_DOPPLER_HZ[0] / grid.fc_meas_hz   # dtau/dt for each target nu
    r9 = REL_DOPPLER_HZ[1] / grid.fc_meas_hz
    snaps = [
        _write(OUT_DIR, "equal_static_a", grid, 0.0),
        _write(OUT_DIR, "equal_static_b", grid, 0.0),
        _write(OUT_DIR, "equal_doppler_5hz", grid, r5),
        _write(OUT_DIR, "equal_doppler_9hz", grid, r9),
    ]
    for i, s in enumerate(snaps):
        s["index"] = i
        s["fc_meas_hz"] = grid.fc_meas_hz
        s["v0_m_s"] = 0.0

    # Sanity: max echo delay stays within the channel limit over 30 s.
    max_echo = DIRECT_DELAY_S + ECHO_DELAY_S + r9 * grid.duration_s

    manifest = {
        "dataset": "BCH1_fading",
        "note": "S0-v2 deep-notch test; equal-amplitude two-tap. snap0/1 "
                "static (fixed deep notch), snap2/3 Doppler-swept notch "
                "(time-selective fading).",
        "symbol_s": 1.0 / 250.0,
        "record_dt_s": grid.dt_s,
        "record_duration_s": grid.duration_s,
        "mean_v0_m_s": 0.0,
        "snapshots": snaps,
    }
    (OUT_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2))

    for s in snaps:
        chk = read_octt(OUT_DIR / s["octt_file"])
        print(f"snap{s['index']} {s['kind']:16s} taps={chk.tap_count} "
              f"amp={s['amp_each']:.3f}x2 echo={s['echo_delay_s']*1e3:.0f}ms "
              f"rel_doppler={s['rel_doppler_hz']:+.1f}Hz "
              f"max_echo={chk.delays.max()*1e3:.2f}ms")
    print(f"  (max echo delay over record = {max_echo*1e3:.2f} ms, "
          f"limit 200 ms)")
    print(f"wrote {OUT_DIR / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
