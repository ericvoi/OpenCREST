"""Experiment 4 — channel-replay validation.

Synthetic closed loop: a known three-tap trajectory (moving tap spacings,
one fading tap) is written to an ``.octt`` file and replayed by
openCREST. Modem A fires a sequence of firmware LFM chirps; each chirp
probes the recorded channel at an advancing record time. Offline
cross-correlation recovers per-chirp tap structure, which is compared
against the trajectory ground truth.

Because the replay clock advances by exact message time (not wall
clock), the analysis first infers each chirp's record time from the
spacing of the first moving tap, then validates the remaining structure
at that inferred time:

  - second tap-spacing residual <= 1 sample (2 us)
  - relative tap amplitudes within 1 dB
  - inferred record times strictly increasing across chirps

Optionally ``--watermark FILE.mat`` converts a real TVIR first and
replays that instead (structural comparison only; no ground truth).

Outputs (under ``--out``, default ``experiments/results/exp4/``):

  trajectory.octt
  <cell_id>/scenario.yaml, WAVs, CDC logs
  chirp_residuals.csv
  fig_replay_validation.pdf
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from experiments.lib import ir_estimator as ir
from experiments.lib import plotting
from experiments.lib.cdc_console import CdcConsole
from experiments.lib.runner import CellHandle, Sweep
from experiments.lib.tap_trajectory import catmull_rom_uniform, write_octt

REPO = Path(__file__).resolve().parents[1]
TEMPLATE = REPO / "experiments" / "configs" / "exp4" / "exp4_replay.yaml.j2"

FS_HZ = 500_000.0
SOUND_SPEED_M_S = 1500.0
DEFAULT_RANGE_M = 300.0
DEFAULT_N_CHIRPS = 8
DEFAULT_CHIRP_SPACING_S = 2.0
DEFAULT_CHANNEL_GAIN_DB = -50.0
SIGTERM_GRACE_S = 6.0

# ---------------------------------------------------------------------------
# Ground-truth trajectory
# ---------------------------------------------------------------------------

# Three taps over a 60 s record (dt = 0.1 s):
#   tap0: direct, slowly shrinking delay (base excess 2 -> 1 ms)
#   tap1: tap0 + 4 ms widening to + 5 ms  -> per-chirp record-time probe
#   tap2: tap0 + 9 ms narrowing to + 7 ms, fading out after 40 s
TRAJ_DT_S = 0.1
TRAJ_DURATION_S = 60.0
SPACING1_AT = lambda t: 0.004 + 0.001 * (t / TRAJ_DURATION_S)      # noqa: E731
SPACING2_AT = lambda t: 0.009 - 0.002 * (t / TRAJ_DURATION_S)      # noqa: E731
AMP1_REL_DB = -4.0
AMP2_REL_DB = -10.0


def make_validation_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Write the ground-truth .octt; returns (delays, amps) grids."""
    frames = int(round(TRAJ_DURATION_S / TRAJ_DT_S)) + 1
    t = np.arange(frames) * TRAJ_DT_S
    tap0 = 0.002 - 0.001 * (t / TRAJ_DURATION_S)
    delays = np.stack([tap0,
                       tap0 + SPACING1_AT(t),
                       tap0 + SPACING2_AT(t)], axis=1)
    a1 = 10.0 ** (AMP1_REL_DB / 20.0)
    a2 = 10.0 ** (AMP2_REL_DB / 20.0)
    amps = np.stack([
        np.ones(frames),
        np.full(frames, a1),
        np.where(t < 40.0, a2, np.where(t < 42.0, a2 * (42.0 - t) / 2.0, 0.0)),
    ], axis=1)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_octt(path, TRAJ_DT_S, 30e3, delays, amps)
    return delays, amps


def record_time_from_spacing1(spacing1_s: float) -> float:
    """Invert SPACING1_AT (linear, monotonic)."""
    return (spacing1_s - 0.004) / 0.001 * TRAJ_DURATION_S


# ---------------------------------------------------------------------------
# Per-chirp analysis
# ---------------------------------------------------------------------------

@dataclass
class ChirpMeasurement:
    chirp_index: int
    record_time_s: float          # inferred from tap-1 spacing
    spacing1_samples: float       # measured tap1 - tap0
    spacing2_samples: float       # measured tap2 - tap0 (nan if faded out)
    spacing2_expected: float      # ground truth at the inferred time
    spacing2_residual: float      # |measured - expected| (nan if faded)
    amp1_rel_db: float            # measured tap1/tap0 (dB)
    amp2_rel_db: float            # measured tap2/tap0 (dB, nan if faded)


def measure_chirp(tx_wav: Path, rx_wav: Path, chirp_index: int,
                  *,
                  fs_hz: float = FS_HZ,
                  max_delay_s: float = 0.020,
                  ) -> ChirpMeasurement | None:
    """IR-estimate one chirp and reduce it to tap-structure numbers."""
    delay_samples, mag = ir.estimate_ir(
        tx_wav, rx_wav, fs_hz=fs_hz, max_delay_s=max_delay_s)

    def peak_in(lo_s: float, hi_s: float) -> tuple[float, float]:
        lo = max(0, int(round(lo_s * fs_hz)))
        hi = min(mag.size - 1, int(round(hi_s * fs_hz)))
        if hi <= lo:
            return float("nan"), float("nan")
        k = lo + int(np.argmax(mag[lo:hi + 1]))
        # Parabolic sub-sample refinement (spacing tolerances are at the
        # single-sample level, so integer peak positions are too coarse).
        off = 0.0
        if 0 < k < mag.size - 1:
            denom = mag[k - 1] - 2.0 * mag[k] + mag[k + 1]
            if denom < 0.0:
                off = float(np.clip(
                    0.5 * (mag[k - 1] - mag[k + 1]) / denom, -0.5, 0.5))
        return float(k) + off, float(mag[k])

    # estimate_ir anchors lag 0 at the strongest peak = tap 0.
    amp0 = float(mag[0])
    # tap1 spacing sweeps 4 -> 5 ms, tap2 sweeps 9 -> 7 ms over the record.
    s1, a1 = peak_in(0.0035, 0.0055)
    s2, a2 = peak_in(0.0065, 0.0095)
    if np.isnan(s1):
        return None

    spacing1_s = s1 / fs_hz
    t_rec = record_time_from_spacing1(spacing1_s)
    expected2 = SPACING2_AT(max(t_rec, 0.0)) * fs_hz

    faded = np.isnan(s2) or a2 < 0.05 * amp0 or t_rec > 40.0
    return ChirpMeasurement(
        chirp_index       = chirp_index,
        record_time_s     = t_rec,
        spacing1_samples  = s1,
        spacing2_samples  = float("nan") if faded else s2,
        spacing2_expected = expected2,
        spacing2_residual = float("nan") if faded else abs(s2 - expected2),
        amp1_rel_db       = 20.0 * np.log10(a1 / amp0),
        amp2_rel_db       = (float("nan") if faded
                             else 20.0 * np.log10(a2 / amp0)),
    )


def analyse_cell(cell_dir: Path,
                 *,
                 tx_modem_serial: str = "OA-2-1",
                 rx_modem_serial: str = "OA-2-2",
                 min_rx_bytes: int = 200_000,
                 ) -> list[ChirpMeasurement]:
    """Measure every chirp-carrying RX session in a cell directory.

    RX sessions are matched to chirps in file order; sessions below
    ``min_rx_bytes`` are HIL-init/idle stubs and are skipped.
    """
    tx_path = cell_dir / f"{tx_modem_serial}_tx.wav"
    if not tx_path.is_file():
        return []
    sessions = [p for p in sorted(cell_dir.glob(f"{rx_modem_serial}_rx_*.wav"))
                if p.stat().st_size >= min_rx_bytes]
    out: list[ChirpMeasurement] = []
    for idx, rx_path in enumerate(sessions):
        m = measure_chirp(tx_path, rx_path, idx)
        if m is not None:
            out.append(m)
    return out


def check_measurements(ms: list[ChirpMeasurement],
                       *,
                       max_spacing_residual_samples: float = 1.0,
                       max_amp_residual_db: float = 1.0,
                       ) -> list[str]:
    """Return a list of failure strings (empty = all assertions pass)."""
    failures: list[str] = []
    if not ms:
        return ["no chirp measurements"]
    for m in ms:
        if not np.isnan(m.spacing2_residual) \
                and m.spacing2_residual > max_spacing_residual_samples:
            failures.append(
                f"chirp {m.chirp_index}: tap2 spacing residual "
                f"{m.spacing2_residual:.2f} samples")
        if abs(m.amp1_rel_db - AMP1_REL_DB) > max_amp_residual_db:
            failures.append(
                f"chirp {m.chirp_index}: tap1 relative amplitude "
                f"{m.amp1_rel_db:.2f} dB (expected {AMP1_REL_DB:.2f})")
        if not np.isnan(m.amp2_rel_db) \
                and abs(m.amp2_rel_db - AMP2_REL_DB) > max_amp_residual_db:
            failures.append(
                f"chirp {m.chirp_index}: tap2 relative amplitude "
                f"{m.amp2_rel_db:.2f} dB (expected {AMP2_REL_DB:.2f})")
    times = [m.record_time_s for m in ms]
    if any(b <= a for a, b in zip(times, times[1:])):
        failures.append(f"record times not strictly increasing: {times}")
    return failures


def write_residuals_csv(ms: list[ChirpMeasurement], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["chirp", "record_time_s",
                    "spacing1_samples", "spacing2_samples",
                    "spacing2_expected", "spacing2_residual_samples",
                    "amp1_rel_db", "amp2_rel_db"])
        for m in ms:
            w.writerow([m.chirp_index, f"{m.record_time_s:.3f}",
                        f"{m.spacing1_samples:.3f}",
                        f"{m.spacing2_samples:.3f}",
                        f"{m.spacing2_expected:.3f}",
                        f"{m.spacing2_residual:.3f}",
                        f"{m.amp1_rel_db:.3f}", f"{m.amp2_rel_db:.3f}"])


def render_residual_plot(ms: list[ChirpMeasurement], savepath: Path) -> None:
    if not ms:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plotting.apply_paper_style()
    t = np.asarray([m.record_time_s for m in ms])
    meas = np.asarray([m.spacing2_samples for m in ms]) / FS_HZ * 1000.0
    exp = np.asarray([m.spacing2_expected for m in ms]) / FS_HZ * 1000.0
    fig, ax = plt.subplots(figsize=(4.0, 2.8))
    ax.plot(t, exp, "-", label="trajectory ground truth")
    ax.plot(t, meas, "o", ms=4, label="measured")
    ax.set_xlabel("Inferred record time (s)")
    ax.set_ylabel("Tap 2 spacing (ms)")
    ax.legend()
    plotting.save_figure(fig, savepath)


# ---------------------------------------------------------------------------
# Hardware run (mirrors exp1's chirp driver, N chirps per cell)
# ---------------------------------------------------------------------------

class _MultiChirpDriver:
    READY_PATTERN = re.compile(r"Simulation running")

    def __init__(self, modem_a_serial: str, modem_b_serial: str,
                 *, n_chirps: int, spacing_s: float,
                 cdc_factory: Callable[..., CdcConsole] = CdcConsole.attach,
                 ready_timeout_s: float = 8.0) -> None:
        self.modem_a_serial = modem_a_serial
        self.modem_b_serial = modem_b_serial
        self.n_chirps = n_chirps
        self.spacing_s = spacing_s
        self._cdc_factory = cdc_factory
        self._ready_timeout = ready_timeout_s
        self._consoles: dict[str, list[CdcConsole]] = {}

    def pre_run(self, handle: CellHandle) -> None:
        self._wait_for_ready(handle.cell_dir / "stdout.log")
        cdc_a = self._cdc_factory(
            modem_id="modem_a", usb_serial=self.modem_a_serial,
            log_path=handle.cell_dir / "modem_a_cdc.log")
        cdc_b = self._cdc_factory(
            modem_id="modem_b", usb_serial=self.modem_b_serial,
            log_path=handle.cell_dir / "modem_b_cdc.log")
        self._consoles[handle.cell_id] = [cdc_a, cdc_b]
        for _ in range(self.n_chirps):
            cdc_a.send_chirp_tx()
            time.sleep(self.spacing_s)

    def post_run(self, handle: CellHandle) -> None:
        for c in self._consoles.pop(handle.cell_id, []):
            try:
                c.detach()
            except Exception:
                pass

    def _wait_for_ready(self, stdout: Path) -> None:
        deadline = time.monotonic() + self._ready_timeout
        while time.monotonic() < deadline:
            if stdout.is_file() and self.READY_PATTERN.search(
                    stdout.read_text(errors="replace")):
                return
            time.sleep(0.1)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--binary", default=str(REPO / "build" / "openCREST"))
    ap.add_argument("--out", type=Path,
                    default=REPO / "experiments" / "results" / "exp4")
    ap.add_argument("--modem-a-serial", default="OA-2-1")
    ap.add_argument("--modem-b-serial", default="OA-2-2")
    ap.add_argument("--n-chirps", type=int, default=DEFAULT_N_CHIRPS)
    ap.add_argument("--chirp-spacing-s", type=float,
                    default=DEFAULT_CHIRP_SPACING_S)
    ap.add_argument("--channel-gain-db", type=float,
                    default=DEFAULT_CHANNEL_GAIN_DB)
    ap.add_argument("--watermark", type=Path, default=None,
                    help="replay a converted Watermark TVIR instead of the "
                         "synthetic trajectory (structural run: analysis "
                         "CSVs are produced but ground-truth assertions "
                         "are skipped)")
    args = ap.parse_args(argv)

    out_dir: Path = args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    octt_path = out_dir / "trajectory.octt"
    if args.watermark is not None:
        from experiments.lib import watermark as wm
        from experiments.lib.tap_trajectory import write_octt as _write
        result = wm.convert(wm.load_tvir(args.watermark))
        _write(octt_path, result.dt_s, result.fc_meas_hz,
               result.delays, result.amplitudes)
        print(f"[exp4] converted {args.watermark} "
              f"(captured energy {result.captured_energy_fraction:.3f})")
    else:
        make_validation_trajectory(octt_path)

    driver = _MultiChirpDriver(
        args.modem_a_serial, args.modem_b_serial,
        n_chirps=args.n_chirps, spacing_s=args.chirp_spacing_s)

    duration_s = 10.0 + args.n_chirps * args.chirp_spacing_s
    results = Sweep(
        template_path   = TEMPLATE,
        parameters      = {"seed": [0]},
        extra_params    = dict(
            trajectory_file = str(octt_path.resolve()),
            range_m         = DEFAULT_RANGE_M,
            modem_a_serial  = args.modem_a_serial,
            modem_b_serial  = args.modem_b_serial,
            channel_gain_db = args.channel_gain_db,
        ),
        binary          = args.binary,
        out_dir         = out_dir,
        duration_s      = duration_s,
        parallel        = 1,
        sigterm_grace_s = SIGTERM_GRACE_S,
        pre_run         = driver.pre_run,
        post_run        = driver.post_run,
    ).run()

    cell_dirs = [p for p in out_dir.iterdir()
                 if p.is_dir() and (p / "scenario.yaml").is_file()]
    ms: list[ChirpMeasurement] = []
    for cd in sorted(cell_dirs):
        ms.extend(analyse_cell(
            cd, tx_modem_serial=args.modem_a_serial,
            rx_modem_serial=args.modem_b_serial))

    write_residuals_csv(ms, out_dir / "chirp_residuals.csv")
    render_residual_plot(ms, out_dir / "fig_replay_validation.pdf")

    if args.watermark is not None:
        print(f"[exp4] structural run complete: {len(ms)} chirps analysed")
        return 0 if ms else 1

    failures = check_measurements(ms)
    for f in failures:
        print(f"[exp4] FAIL: {f}", file=sys.stderr)
    n_bad_cells = sum(1 for r in results if not r.ok)
    if not failures and n_bad_cells == 0:
        print(f"[exp4] PASS: {len(ms)} chirps validated against the "
              f"trajectory ground truth")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
