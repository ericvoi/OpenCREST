"""Build a single-tap (multipath-free) .octt + manifest for the sync
diagnostic: one constant arrival, no multipath, no fading, no Doppler.

Run exp5 against this and against the real BCH1 ensemble at the same
gains. If the BER floor is gone here but present in the multipath
ensemble, the floor is multipath/sync-induced. If it survives, it is
"something more general" (frequency offset, intra-packet timing drift).

The grid (dt_s, fc_meas_hz, frame_count) is inherited from an existing
snapshot so the replay record is long enough that every EVAL message
stays inside it (offset_s = record_dt_s plays in the interior).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from experiments.lib.tap_trajectory import read_octt, write_octt

REPO = Path(__file__).resolve().parent.parent
TEMPLATE_OCTT = REPO / "experiments/results/ensembles/BCH1/BCH1_snap00.octt"
OUT_DIR = REPO / "experiments/results/ensembles/BCH1_singletap"

# Constant excess delay for the lone tap. Kept at the converter's
# Catmull-Rom guard floor rather than 0 so max_delay_s is nonzero and the
# C++ replay sizing takes its normal (non-degenerate) path. As the only
# path it is just a constant bulk offset the modem syncs through.
GUARD_DELAY_S = 2e-5


def main() -> int:
    grid = read_octt(TEMPLATE_OCTT)
    frame_count = grid.frame_count
    dt_s = grid.dt_s
    fc_meas_hz = grid.fc_meas_hz

    # One tap, constant over the whole record: amplitude 1.0 matches the
    # peak-normalised level of the multipath ensemble, so the dominant-path
    # SNR is the same at a given gain_db.
    delays = np.full((frame_count, 1), GUARD_DELAY_S, dtype=np.float64)
    amplitudes = np.ones((frame_count, 1), dtype=np.float32)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    octt_path = OUT_DIR / "BCH1_singletap.octt"
    write_octt(octt_path, dt_s, fc_meas_hz, delays, amplitudes)

    manifest = {
        "dataset": "BCH1_singletap",
        "note": "single constant tap; multipath/fading/Doppler removed "
                "(sync diagnostic)",
        "record_dt_s": dt_s,
        "record_duration_s": (frame_count - 1) * dt_s,
        "mean_v0_m_s": 0.0,
        "snapshots": [
            {
                "index": 0,
                "octt_file": "BCH1_singletap.octt",
                "fc_meas_hz": fc_meas_hz,
                "v0_m_s": 0.0,
                "tap_delay_s": GUARD_DELAY_S,
                "tap_amplitude": 1.0,
            }
        ],
    }
    (OUT_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2))

    # Read back to confirm it validates through the same loader C++ mirrors.
    check = read_octt(octt_path)
    print(f"wrote {octt_path}")
    print(f"  taps={check.tap_count} frames={check.frame_count} "
          f"dt={check.dt_s}s dur={check.duration_s}s fc_meas={check.fc_meas_hz}Hz")
    print(f"  delay(const)={check.delays.max()*1e6:.1f} us  "
          f"amp(const)={check.amplitudes.max():.3f}")
    print(f"wrote {OUT_DIR / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
