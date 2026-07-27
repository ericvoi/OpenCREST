"""Convert a Watermark-style TVIR (.mat) to an openCREST tap-trajectory
file (.octt).

Two modes:

* default — time-varying tap tracks (peak association across snapshots,
  phase-derived fine delay);
* ``--quasi-static`` — one frozen tap set per window, estimated from the
  windowed scattering function: each delay-Doppler peak becomes a tap
  {delay, amplitude, Doppler}, emitted as linear delay ramps.

Examples:

    experiments/.venv/bin/python -m experiments.convert_watermark \
        NOF1_001.mat nof1_001.octt --max-taps 16 --prominence-db -25

    experiments/.venv/bin/python -m experiments.convert_watermark \
        BCH1_001.mat bch1_snap0.octt --quasi-static \
        --start-s 0 --duration-s 20 --plot

The converter reports the captured-energy fraction (how much of the
picked-peak energy the kept taps carry — diffuse scattering that never
resolves into a discrete arrival is not counted) and the peak level it
normalised away (compensate via the channel's gain_db).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from experiments.lib import quasi_static, watermark
from experiments.lib.tap_trajectory import MAX_TAPS, write_octt


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert a Watermark TVIR .mat to an .octt tap "
                    "trajectory for openCREST channel replay.")
    ap.add_argument("input", type=Path, help=".mat TVIR file")
    ap.add_argument("output", type=Path, help=".octt output path")
    ap.add_argument("--max-taps", type=int, default=MAX_TAPS,
                    help=f"strongest tracks to keep (default {MAX_TAPS})")
    ap.add_argument("--prominence-db", type=float, default=-30.0,
                    help="peak-pick threshold below the global |h| max "
                         "(default -30)")
    ap.add_argument("--gate-bins", type=float, default=3.0,
                    help="track-association delay gate in bins (default 3)")
    ap.add_argument("--min-track-frames", type=int, default=3,
                    help="discard tracks shorter than this (default 3)")
    ap.add_argument("--smooth-frames", type=int, default=5,
                    help="Savitzky-Golay window on delay/amplitude tracks "
                         "(default 5; larger = smoother, less fast fading)")
    ap.add_argument("--start-s", type=float, default=0.0,
                    help="record time to start at (default 0); in "
                         "quasi-static mode this is the window start")
    ap.add_argument("--duration-s", type=float, default=None,
                    help="seconds to keep (default: to record end); in "
                         "quasi-static mode this is the window length")
    ap.add_argument("--plot", action="store_true",
                    help="write a QA figure (<output>.qa.pdf): extracted "
                         "trajectories over the |h| waterfall + amplitude "
                         "tracks, or the scattering function + picked "
                         "peaks in quasi-static mode")
    ap.add_argument("--quasi-static", action="store_true",
                    help="freeze one window into a static tap set with "
                         "per-tap Doppler (linear delay ramps) instead of "
                         "tracking time-varying taps")
    ap.add_argument("--record-duration-s", type=float, default=30.0,
                    help="quasi-static: duration of the emitted record "
                         "(default 30; set scenario offset_s to the frame "
                         "interval and keep messages inside the record)")
    ap.add_argument("--record-dt-s", type=float, default=1.0,
                    help="quasi-static: frame interval of the emitted "
                         "record (default 1.0)")
    args = ap.parse_args(argv)

    tvir = watermark.load_tvir(args.input)

    if args.quasi_static:
        return _convert_quasi_static(args, tvir)

    if args.start_s > 0.0 or args.duration_s is not None:
        i0 = int(round(args.start_s * tvir.fs_t))
        i1 = (tvir.h.shape[0] if args.duration_s is None
              else i0 + int(round(args.duration_s * tvir.fs_t)))
        tvir.h = tvir.h[i0:i1]
        if tvir.h.shape[0] < 2:
            print("error: trim window leaves fewer than 2 snapshots",
                  file=sys.stderr)
            return 1

    result = watermark.convert(
        tvir,
        max_taps=args.max_taps,
        prominence_db=args.prominence_db,
        gate_bins=args.gate_bins,
        min_track_frames=args.min_track_frames,
        smooth_frames=args.smooth_frames,
    )

    write_octt(args.output, result.dt_s, result.fc_meas_hz,
               result.delays, result.amplitudes)

    frames, taps = result.delays.shape
    print(f"{args.output}: {taps} taps x {frames} frames, "
          f"dt {result.dt_s * 1000:.1f} ms "
          f"({(frames - 1) * result.dt_s:.1f} s record), "
          f"fc_meas {result.fc_meas_hz / 1000:.1f} kHz")
    print(f"  captured-energy fraction: "
          f"{result.captured_energy_fraction:.3f}")
    print(f"  peak normalisation removed: "
          f"{result.normalization_db:+.1f} dB "
          f"(set the channel's gain_db to place the absolute level)")

    if args.plot:
        qa_path = args.output.with_suffix(args.output.suffix + ".qa.pdf")
        watermark.render_conversion_qa(tvir, result, qa_path)
        print(f"  QA figure: {qa_path}")
    return 0


def _convert_quasi_static(args: argparse.Namespace,
                          tvir: watermark.TVIRData) -> int:
    try:
        taps = quasi_static.extract_quasi_static(
            tvir,
            start_s=args.start_s,
            duration_s=args.duration_s,
            max_taps=args.max_taps,
            prominence_db=args.prominence_db,
        )
        traj = quasi_static.quasi_static_to_trajectories(
            taps,
            record_duration_s=args.record_duration_s,
            dt_s=args.record_dt_s,
        )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    write_octt(args.output, traj.dt_s, traj.fc_meas_hz,
               traj.delays, traj.amplitudes)

    print(f"{args.output}: {taps.tap_count} quasi-static taps, "
          f"{traj.delays.shape[0]} frames x {traj.dt_s:.1f} s "
          f"({args.record_duration_s:.1f} s record), "
          f"fc_meas {taps.fc_meas_hz / 1000:.1f} kHz, "
          f"window [{taps.window_start_s:.1f}, "
          f"{taps.window_start_s + taps.window_duration_s:.1f}] s")
    print(f"  captured-energy fraction: "
          f"{taps.captured_energy_fraction:.3f}")
    print(f"  peak normalisation removed: {taps.normalization_db:+.1f} dB "
          f"(set the channel's gain_db to place the absolute level)")
    print(f"  mean Doppler v0 {taps.v0_m_s:+.3f} m/s reinstalled as a "
          f"common delay ramp; Doppler resolution "
          f"{taps.doppler_resolution_hz:.3f} Hz")
    print(f"  {'tap':>3} {'delay (ms)':>11} {'amp (dB)':>9} "
          f"{'Doppler (Hz)':>13}")
    for k in range(taps.tap_count):
        amp_db = 20.0 * math.log10(taps.amplitudes[k])
        print(f"  {k:>3} {taps.delays_s[k] * 1000:>11.3f} {amp_db:>9.1f} "
              f"{taps.doppler_hz[k]:>+13.3f}")

    if args.plot:
        qa_path = args.output.with_suffix(args.output.suffix + ".qa.pdf")
        quasi_static.render_quasi_static_qa(tvir, taps, qa_path)
        print(f"  QA figure: {qa_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
