"""Build the S0 two-tap discriminator .octt files that separate a
*sync-bias* floor from an *ISI* floor.

Two channels, each a direct path plus one delayed echo. The only
difference is which arrival is stronger:

* ``direct_dominant`` — strong direct + weaker late echo. The firmware's
  global-max sync still picks the direct path, so timing is CORRECT and
  this isolates pure inter-symbol interference from the echo.
* ``strong_echo`` — weak direct + stronger late echo. Global-max sync
  locks onto the LATE echo, so this isolates the sync-timing bias.

Reading the sweep:
  * strong_echo floors but direct_dominant is clean  -> sync bias; the
    first-arrival direction is worth pursuing.
  * direct_dominant ALSO floors                      -> ISI-limited; no
    sync change fixes it (need slower baud / wider hops / equaliser).

Grid (dt, fc_meas, frame_count) is inherited from an existing snapshot
so the record outlasts every EVAL message.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from experiments.lib.tap_trajectory import read_octt, write_octt

REPO = Path(__file__).resolve().parent.parent
TEMPLATE_OCTT = REPO / "experiments/results/ensembles/BCH1/BCH1_snap00.octt"
OUT_DIR = REPO / "experiments/results/ensembles/BCH1_s0"

DIRECT_DELAY_S = 2e-5     # direct path, at the Catmull-Rom guard floor
SYMBOL_S = 1.0 / 250.0    # baud 250 -> 4 ms symbol
# Echo excess delays, all SUB-symbol so the arrivals are UNRESOLVED and
# each symbol's energy smears into the next window ("the symbol seems to
# last longer than it is"). A >symbol echo is a clean resolved copy the
# demod just decodes shifted, which is why the 5 ms version showed no
# penalty. 25/50/75 % of the 4 ms symbol maps where the floor kicks in.
ECHO_DELAYS_S = [1e-3, 2e-3, 3e-3]
STRONG, WEAK = 1.0, 0.5   # 6 dB split — sets which side the peak centroid


def _write_two_tap(out_dir: Path, name: str, dt_s: float, fc_meas_hz: float,
                   frame_count: int, echo_delay_s: float,
                   amp_direct: float, amp_echo: float) -> dict:
    delays = np.empty((frame_count, 2), dtype=np.float64)
    delays[:, 0] = DIRECT_DELAY_S
    delays[:, 1] = DIRECT_DELAY_S + echo_delay_s
    amps = np.empty((frame_count, 2), dtype=np.float32)
    amps[:, 0] = amp_direct
    amps[:, 1] = amp_echo

    path = out_dir / f"{name}.octt"
    write_octt(path, dt_s, fc_meas_hz, delays, amps)
    return {
        "octt_file": f"{name}.octt",
        "kind": name,
        "echo_delay_s": echo_delay_s,
        "echo_frac_symbol": echo_delay_s / SYMBOL_S,
        "direct": {"delay_s": DIRECT_DELAY_S, "amplitude": amp_direct},
        "echo":   {"delay_s": DIRECT_DELAY_S + echo_delay_s,
                   "amplitude": amp_echo},
    }


def main() -> int:
    grid = read_octt(TEMPLATE_OCTT)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # For each sub-symbol delay: direct-dominant (energy centroid early,
    # isolates ISI with best-effort sync) and strong-echo (centroid late,
    # adds the sync-timing bias on top of the same ISI).
    snaps: list[dict] = []
    for tau in ECHO_DELAYS_S:
        tag = f"{round(tau * 1e3)}ms"
        snaps.append(_write_two_tap(
            OUT_DIR, f"BCH1_direct_dominant_{tag}", grid.dt_s,
            grid.fc_meas_hz, grid.frame_count, tau, STRONG, WEAK))
        snaps.append(_write_two_tap(
            OUT_DIR, f"BCH1_strong_echo_{tag}", grid.dt_s,
            grid.fc_meas_hz, grid.frame_count, tau, WEAK, STRONG))
    for i, s in enumerate(snaps):
        s["index"] = i
        s["fc_meas_hz"] = grid.fc_meas_hz
        s["v0_m_s"] = 0.0

    manifest = {
        "dataset": "BCH1_s0",
        "note": "S0 sync-bias vs ISI discriminator, SUB-symbol echoes "
                "(symbol=4 ms at baud 250). Pairs per delay: "
                "direct_dominant (isolates ISI, best-effort sync) vs "
                "strong_echo (adds late-centroid sync bias).",
        "symbol_s": SYMBOL_S,
        "record_dt_s": grid.dt_s,
        "record_duration_s": (grid.frame_count - 1) * grid.dt_s,
        "mean_v0_m_s": 0.0,
        "snapshots": snaps,
    }
    (OUT_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2))

    for s in snaps:
        chk = read_octt(OUT_DIR / s["octt_file"])
        print(f"snap{s['index']:2d} {s['kind']:28s} taps={chk.tap_count} "
              f"echo={s['echo_delay_s']*1e3:.0f}ms "
              f"({s['echo_frac_symbol']*100:.0f}% symbol) "
              f"a_direct={s['direct']['amplitude']} "
              f"a_echo={s['echo']['amplitude']}")
    print(f"wrote {OUT_DIR / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
