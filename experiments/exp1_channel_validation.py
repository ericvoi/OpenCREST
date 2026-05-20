"""Experiment 1 — fixed-range channel-model validation (paper §4.2).

Sweeps 17 ranges × 3 seeds = 51 stationary cells through the geometric
channel. A single Session-A LFM chirp is launched on modem A; modem B's
RX captures the channel-modified signal. Offline cross-correlation
recovers the impulse response; analytical method-of-images delays for
direct / surface / bottom paths are overlaid on the resulting waterfall.

Outputs (under ``--out``, default ``experiments/results/exp1/``):

  <cell_id>/scenario.yaml
  <cell_id>/<name>_summary.json
  <cell_id>/modem_a_tx.wav
  <cell_id>/modem_b_rx_001.wav
  <cell_id>/modem_*_cdc.log

  sweep_index.csv
  residuals.csv                    one row per (range, seed, path)
  fig_validation.pdf               waterfall + analytical overlay

Use ``--seeds N`` to pick the per-range seed count (default 3, the
paper figure). The IR window, chirp wait, and analysis range can be
overridden via flags.

Real hardware is required for the actual sweep (two OpenAquatix modems
with the configured USB serials). For the smoke test against
``_stub_binary.py`` see ``experiments/tests/test_exp1_smoke.py``.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import yaml

from experiments.lib import analytical_taps as at
from experiments.lib import ir_estimator as ir
from experiments.lib import plotting
from experiments.lib.cdc_console import CdcConsole
from experiments.lib.runner import CellHandle, Sweep


REPO = Path(__file__).resolve().parents[1]
TEMPLATE = REPO / "experiments" / "configs" / "exp1" / "exp1_validation.yaml.j2"

DEFAULT_RANGES_M = list(range(200, 1001, 50))    # 17 entries: 200, 250, ..., 1000
DEFAULT_SEEDS = (0, 1, 2)
DEFAULT_FS_HZ = 500_000.0
DEFAULT_MAX_DELAY_S = 0.035                       # accommodates surface at R=200

# Geometry defaults (kept in sync with the YAML template).
WATER_DEPTH_M     = 120.0
SOURCE_DEPTH_M    = 50.0
RECEIVER_DEPTH_M  = 100.0
SOUND_SPEED_M_S   = 1500.0

# Worst-case per-cell timing budget:
#   • simulator init                                    ~1.0 s
#   • CDC navigation + chirp command                    ~0.3 s
#   • LFM chirp duration (Session A firmware probe)     ~0.05 s
#   • one-way propagation at R=1000 m / 1500 m·s⁻¹      ~0.67 s
#   • multipath excess (≤30 ms across the sweep)        ~0.05 s
#   • RX-session draining + finalize slack              ~1.5 s
#   • shutdown safety margin                            ~1.5 s
# Set to 5.0 s after empirical evidence that 4.0 s sometimes truncated the
# RX WAV with the chirp landing in the last 100 ms (the StreamLogger
# finalize beat the chirp arrival to disk).
DEFAULT_CELL_DURATION_S = 5.0
SIGTERM_GRACE_S = 6.0          # Simulator's emit_run_summary takes a moment
DEFAULT_RETRIES = 2            # Transient USB hiccups on modem-open; see usb_transport.cpp
# Additive trim on the channel chain. With noise.disable=true the Phase-C
# receiver auto-boost still runs (~+49 dB on the current modem cal), so the
# chirp at 0.5 source amplitude saturates the receiver DAC by ~50× without
# this trim. -50 dB lands worst-case constructive multipath peaks at ~0.8
# of full scale at R=200, and the direct path at ~0.04 at R=1000 — both
# inside the linear range and well above 12-bit quantization noise.
DEFAULT_CHANNEL_GAIN_DB = -50.0


# ---------------------------------------------------------------------------
# CDC orchestration
# ---------------------------------------------------------------------------

class _ChirpDriver:
    """Per-cell ``pre_run``/``post_run`` glue:

    * Waits until the simulator prints its readiness banner to stdout.
    * Attaches a CDC console to modem A and modem B.
    * Issues the firmware LFM chirp on modem A.

    The CDC consoles stay attached for the lifetime of the cell so the
    firmware's per-message debug prints (which include the chirp ack and
    any RX session boundaries) land in the per-modem ``*_cdc.log`` file
    for later forensics.
    """

    READY_PATTERN = re.compile(r"Simulation running")

    def __init__(self,
                 modem_a_serial: str,
                 modem_b_serial: str,
                 *,
                 cdc_factory: Callable[..., CdcConsole] = CdcConsole.attach,
                 ready_timeout_s: float = 8.0,
                 ) -> None:
        self.modem_a_serial = modem_a_serial
        self.modem_b_serial = modem_b_serial
        self._cdc_factory   = cdc_factory
        self._ready_timeout = ready_timeout_s
        self._consoles: dict[str, list[CdcConsole]] = {}

    def pre_run(self, handle: CellHandle) -> None:
        stdout = handle.cell_dir / "stdout.log"
        self._wait_for_ready(stdout)
        cdc_a = self._cdc_factory(
            modem_id   = "modem_a",
            usb_serial = self.modem_a_serial,
            log_path   = handle.cell_dir / "modem_a_cdc.log",
        )
        cdc_b = self._cdc_factory(
            modem_id   = "modem_b",
            usb_serial = self.modem_b_serial,
            log_path   = handle.cell_dir / "modem_b_cdc.log",
        )
        self._consoles[handle.cell_id] = [cdc_a, cdc_b]
        # The chirp drives normal HIL TX→RX cycling (see feedback_chirp_routing.md);
        # one trigger is enough to exercise the full multipath structure.
        cdc_a.send_chirp_tx()

    def post_run(self, handle: CellHandle) -> None:
        for c in self._consoles.pop(handle.cell_id, []):
            try:
                c.detach()
            except Exception:
                pass

    def _wait_for_ready(self, stdout: Path) -> None:
        deadline = time.monotonic() + self._ready_timeout
        while time.monotonic() < deadline:
            if stdout.is_file():
                try:
                    text = stdout.read_text(errors="replace")
                except OSError:
                    text = ""
                if self.READY_PATTERN.search(text):
                    return
            time.sleep(0.1)
        # Don't raise — the cell will run for its full duration and probably
        # produce no RX; the driver will mark it failed at analysis time.


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

@dataclass
class CellAnalysis:
    cell_id: str
    range_m: float
    seed: int
    delay_samples: np.ndarray
    correlation_mag: np.ndarray         # normalised, peak = 1.0
    measured: dict[str, float]          # path -> measured delay (samples)
    measured_mag: dict[str, float]      # path -> peak magnitude
    analytical: dict[str, float]        # path -> analytical delay (samples)


def analyse_cell(cell_dir: Path,
                 range_m: float,
                 seed: int,
                 *,
                 fs_hz: float = DEFAULT_FS_HZ,
                 max_delay_s: float = DEFAULT_MAX_DELAY_S,
                 tx_modem_serial: str = "OA-2-1",
                 rx_modem_serial: str = "OA-2-2",
                 ) -> CellAnalysis | None:
    """Compute IR + tap residuals for one cell. Returns ``None`` if the
    expected WAV files are missing (cell failed).

    The simulator names WAV files by USB serial (see
    ``modem_registry.cpp::27`` — the Modem object's ``id()`` is the USB
    serial, not the YAML modem id). On the RX side the firmware opens a
    new session every time it enters RX state autonomously; ``rx_001``
    is usually empty because the firmware opens it at HIL-init before
    the chirp lands. We pick the largest non-empty session as the chirp
    carrier — that's the only one that could plausibly contain a 50 ms
    chirp + multipath tail.
    """
    tx_path = cell_dir / f"{tx_modem_serial}_tx.wav"
    rx_path = _largest_rx_session(cell_dir, rx_modem_serial)
    if not tx_path.is_file() or rx_path is None:
        return None

    delay_samples, mag = ir.estimate_ir(
        tx_path, rx_path, fs_hz=fs_hz, max_delay_s=max_delay_s)

    analytical = {
        t.name: t.excess_delay_samples
        for t in at.analytical_taps(
            range_m,
            water_depth_m   = WATER_DEPTH_M,
            source_depth_m  = SOURCE_DEPTH_M,
            receiver_depth_m= RECEIVER_DEPTH_M,
            sound_speed_m_s = SOUND_SPEED_M_S,
            fs_hz           = fs_hz,
        )
    }

    measured: dict[str, float] = {}
    measured_mag: dict[str, float] = {}
    for name, expected in analytical.items():
        # Search a ±1 ms window around the analytical delay; pick the
        # local-max within that window. Robust against any cycle ambiguity
        # in the chirp's autocorrelation main lobe.
        half_window = int(round(0.001 * fs_hz))
        lo = max(0, int(round(expected)) - half_window)
        hi = min(mag.size - 1, int(round(expected)) + half_window)
        if hi <= lo:
            measured[name] = float("nan")
            measured_mag[name] = float("nan")
            continue
        local = mag[lo:hi + 1]
        offset = int(np.argmax(local))
        measured[name] = float(lo + offset)
        measured_mag[name] = float(local[offset])

    return CellAnalysis(
        cell_id         = cell_dir.name,
        range_m         = float(range_m),
        seed            = int(seed),
        delay_samples   = delay_samples,
        correlation_mag = mag,
        measured        = measured,
        measured_mag    = measured_mag,
        analytical      = analytical,
    )


def _largest_rx_session(cell_dir: Path, serial: str) -> Path | None:
    """Pick the largest ``<serial>_rx_NNN.wav`` file (or None if there are none).

    Firmware opens a new RX session every time the modem enters RX state;
    rx_001 is typically empty because it covers the simulator's
    HIL-init transition before the chirp lands. The session that
    contains the chirp + multipath tail will be substantially larger
    than the empty/short ones around it.
    """
    candidates = sorted(cell_dir.glob(f"{serial}_rx_*.wav"))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_size)


def write_residuals_csv(analyses: list[CellAnalysis], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["range_m", "seed", "path",
                    "analytical_delay_samples", "measured_delay_samples",
                    "residual_samples", "xcorr_peak"])
        for a in analyses:
            for name in ("direct", "surface", "bottom"):
                if name not in a.analytical:
                    continue
                ana = a.analytical[name]
                meas = a.measured.get(name, float("nan"))
                mag  = a.measured_mag.get(name, float("nan"))
                resid = abs(meas - ana) if not math.isnan(meas) else float("nan")
                w.writerow([
                    f"{a.range_m:.1f}", a.seed, name,
                    f"{ana:.3f}",
                    "nan" if math.isnan(meas) else f"{meas:.3f}",
                    "nan" if math.isnan(resid) else f"{resid:.3f}",
                    "nan" if math.isnan(mag) else f"{mag:.4f}",
                ])


def render_waterfall(analyses: list[CellAnalysis],
                     savepath: Path,
                     *,
                     fs_hz: float = DEFAULT_FS_HZ,
                     ) -> None:
    """One waterfall row per range (averaged across seeds for that range)."""
    if not analyses:
        return
    by_range: dict[float, list[CellAnalysis]] = {}
    for a in analyses:
        by_range.setdefault(a.range_m, []).append(a)
    ranges = sorted(by_range)

    # Resample every cell to the same delay axis (the first cell's axis).
    n_lags = min(a.correlation_mag.size for a in analyses)
    delays_samples = analyses[0].delay_samples[:n_lags]
    delays_ms = delays_samples / fs_hz * 1000.0

    grid = np.zeros((len(ranges), n_lags), dtype=np.float64)
    for i, R in enumerate(ranges):
        rows = np.stack([a.correlation_mag[:n_lags] for a in by_range[R]])
        grid[i, :] = rows.mean(axis=0)

    fig = plotting.waterfall(
        grid,
        x_axis     = delays_ms,
        y_axis     = np.asarray(ranges),
        cmap       = "viridis",
        x_label    = "Excess delay (ms)",
        y_label    = "Range (m)",
        cbar_label = "|xcorr| (normalised)",
        title      = "",
    )

    delays_per_path: dict[str, list[float]] = {"direct": [], "surface": [], "bottom": []}
    for R in ranges:
        taps = at.analytical_taps(
            R,
            water_depth_m   = WATER_DEPTH_M,
            source_depth_m  = SOURCE_DEPTH_M,
            receiver_depth_m= RECEIVER_DEPTH_M,
            sound_speed_m_s = SOUND_SPEED_M_S,
            fs_hz           = fs_hz,
        )
        by_name = {t.name: t.excess_delay_samples / fs_hz * 1000.0 for t in taps}
        for name in delays_per_path:
            delays_per_path[name].append(by_name.get(name, float("nan")))

    plotting.overlay_tap_lines(
        fig,
        ranges_m        = np.asarray(ranges),
        delays_per_path = {k: np.asarray(v) for k, v in delays_per_path.items()},
        x_unit          = "ms",
    )
    plotting.save_figure(fig, savepath)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", default="experiments/results/exp1",
                   help="output directory for sweep artifacts")
    p.add_argument("--binary",
                   default=str(REPO / "build" / "openCREST"),
                   help="path to the openCREST simulator binary")
    p.add_argument("--seeds", type=int, default=len(DEFAULT_SEEDS),
                   help="number of seeds per range (default 3)")
    p.add_argument("--ranges-m", default=None,
                   help="comma-separated ranges (m) to override the default "
                        "200..1000 step 50 sweep")
    p.add_argument("--modem-a-serial", default="OA-2-1")
    p.add_argument("--modem-b-serial", default="OA-2-2")
    p.add_argument("--duration-s", type=float, default=DEFAULT_CELL_DURATION_S,
                   help="per-cell wall time before SIGTERM (default 5.0)")
    p.add_argument("--retries", type=int, default=DEFAULT_RETRIES,
                   help="extra attempts per cell on failure (default 2)")
    p.add_argument("--channel-gain-db", type=float,
                   default=DEFAULT_CHANNEL_GAIN_DB,
                   help="additive trim on the channel gain chain to keep the "
                        "chirp out of saturation (default -50.0). Tune by "
                        "inspecting the AFE_dB stdout line.")
    p.add_argument("--fs-hz", type=float, default=DEFAULT_FS_HZ)
    p.add_argument("--max-delay-s", type=float, default=DEFAULT_MAX_DELAY_S,
                   help="positive-lag window for the IR estimate (default "
                        "0.035 s = 35 ms, accommodates surface at R=200)")
    p.add_argument("--check-determinism", action="store_true",
                   help="run a reproducibility pre-flight: re-execute one "
                        "cell twice and assert the xcorr peak locations "
                        "match within 1 sample per path. Off by default — "
                        "byte-level identical WAVs are not achievable on "
                        "real HW because firmware-autonomous RX sessions "
                        "have nondeterministic zero-padding around the "
                        "chirp arrival.")
    p.add_argument("--analysis-only", action="store_true",
                   help="don't run the sweep — only post-process artifacts "
                        "already present in --out")
    args = p.parse_args(argv)

    out_dir = Path(args.out).resolve()
    ranges_m = ([float(s) for s in args.ranges_m.split(",")]
                if args.ranges_m else [float(r) for r in DEFAULT_RANGES_M])
    seeds = list(range(int(args.seeds)))

    if not args.analysis_only:
        if args.check_determinism:
            rc = _determinism_preflight(
                out_dir / "_det",
                binary          = args.binary,
                range_m         = ranges_m[len(ranges_m) // 2],   # middle range
                modem_a         = args.modem_a_serial,
                modem_b         = args.modem_b_serial,
                duration_s      = args.duration_s,
                fs_hz           = args.fs_hz,
                max_delay_s     = args.max_delay_s,
                channel_gain_db = args.channel_gain_db,
            )
            if rc != 0:
                return rc

        chirp = _ChirpDriver(args.modem_a_serial, args.modem_b_serial)
        sweep = Sweep(
            template_path     = TEMPLATE,
            parameters        = {"range_m": ranges_m, "seed": seeds},
            extra_params      = dict(
                modem_a_serial  = args.modem_a_serial,
                modem_b_serial  = args.modem_b_serial,
                channel_gain_db = args.channel_gain_db,
            ),
            binary            = args.binary,
            out_dir           = out_dir,
            duration_s        = float(args.duration_s),
            parallel          = 1,           # two modems can't be shared
            sigterm_grace_s   = SIGTERM_GRACE_S,
            retries           = int(args.retries),
            pre_run           = chirp.pre_run,
            post_run          = chirp.post_run,
        )
        results = sweep.run()
        n_fail = sum(1 for r in results if not r.ok)
        if n_fail > 0:
            sys.stderr.write(f"[exp1] {n_fail}/{len(results)} cells failed; "
                             f"continuing into analysis with what landed\n")

    analyses: list[CellAnalysis] = []
    for cell_dir, R, seed in _discover_cells(out_dir):
        a = analyse_cell(
            cell_dir, R, seed,
            fs_hz           = args.fs_hz,
            max_delay_s     = args.max_delay_s,
            tx_modem_serial = args.modem_a_serial,
            rx_modem_serial = args.modem_b_serial,
        )
        if a is None:
            sys.stderr.write(
                f"[exp1] cell {cell_dir.name} (R={R}, seed={seed}) missing "
                f"WAVs; skipped\n")
            continue
        analyses.append(a)

    if not analyses:
        sys.stderr.write("[exp1] no cells analysable — aborting\n")
        return 3

    residuals = out_dir / "residuals.csv"
    write_residuals_csv(analyses, residuals)
    render_waterfall(analyses,
                     out_dir / "fig_validation.pdf",
                     fs_hz=args.fs_hz)

    _print_summary(analyses)
    return 0


def _discover_cells(out_dir: Path) -> list[tuple[Path, float, int]]:
    """Walk ``out_dir`` for cell directories and read each one's
    ``scenario.yaml`` to recover ``(cell_dir, range_m, seed)``.

    Used by the analysis loop instead of recomputing cell ids — the runner
    hashes all ``extra_params`` into the id, so any sweep-time parameter
    that the driver doesn't pass back through to the analysis would break
    a hash-based lookup. Reading the YAML is the source of truth.
    """
    out: list[tuple[Path, float, int]] = []
    for child in sorted(out_dir.iterdir()):
        if not child.is_dir():
            continue
        scenario = child / "scenario.yaml"
        if not scenario.is_file():
            continue
        try:
            doc = yaml.safe_load(scenario.read_text())
        except Exception:
            continue
        if not isinstance(doc, dict):
            continue
        seed = doc.get("random_seed")
        channels = doc.get("channels") or []
        if not channels:
            continue
        first = channels[0] or {}
        range_m = first.get("initial_range_m", first.get("range_m"))
        if seed is None or range_m is None:
            continue
        out.append((child, float(range_m), int(seed)))
    return out


def _determinism_preflight(det_dir: Path,
                            *,
                            binary: str,
                            range_m: float,
                            modem_a: str,
                            modem_b: str,
                            duration_s: float,
                            fs_hz: float,
                            max_delay_s: float,
                            channel_gain_db: float,
                            ) -> int:
    """Soft determinism: run the same cell twice and compare xcorr peak
    locations.

    Why not byte-level: on real HW the raw WAV streams contain varying
    numbers of zero-padding samples around the chirp arrival because the
    firmware autonomously cycles TX/RX with jitter relative to simulator
    startup. The interesting deterministic property — that the recovered
    direct/surface/bottom peak *delays* are stable — is what the paper
    cares about; byte-equality is too strong.
    """
    sys.stderr.write("[exp1] running determinism pre-flight (xcorr-peak)\n")
    def one(suffix: str) -> Path:
        chirp = _ChirpDriver(modem_a, modem_b)
        Sweep(
            template_path   = TEMPLATE,
            parameters      = {"range_m": [range_m], "seed": [0]},
            extra_params    = dict(modem_a_serial=modem_a,
                                   modem_b_serial=modem_b,
                                   channel_gain_db=channel_gain_db),
            binary          = binary,
            out_dir         = det_dir / suffix,
            duration_s      = float(duration_s),
            parallel        = 1,
            sigterm_grace_s = SIGTERM_GRACE_S,
            pre_run         = chirp.pre_run,
            post_run        = chirp.post_run,
            progress        = False,
        ).run()
        return next(p for p in (det_dir / suffix).iterdir() if p.is_dir())

    cell_a = one("a")
    cell_b = one("b")
    a = analyse_cell(cell_a, range_m, seed=0, fs_hz=fs_hz,
                     max_delay_s=max_delay_s,
                     tx_modem_serial=modem_a, rx_modem_serial=modem_b)
    b = analyse_cell(cell_b, range_m, seed=0, fs_hz=fs_hz,
                     max_delay_s=max_delay_s,
                     tx_modem_serial=modem_a, rx_modem_serial=modem_b)
    if a is None or b is None:
        sys.stderr.write("[exp1] determinism check FAILED — at least one "
                         "of the two runs did not produce analysable "
                         "WAVs\n")
        return 4

    drift: list[tuple[str, float, float, float]] = []
    ok = True
    for name in ("direct", "surface", "bottom"):
        ma = a.measured.get(name, float("nan"))
        mb = b.measured.get(name, float("nan"))
        d  = abs(ma - mb)
        drift.append((name, ma, mb, d))
        if math.isnan(d) or d > 1.0:
            ok = False
    sys.stderr.write("[exp1] determinism (xcorr-peak delta, samples):\n")
    for name, ma, mb, d in drift:
        mark = "PASS" if d <= 1.0 else "FAIL"
        sys.stderr.write(f"  [{mark}] {name:7s} a={ma:.2f}  b={mb:.2f}  Δ={d:.2f}\n")
    if not ok:
        sys.stderr.write("[exp1] determinism check FAILED — peak locations "
                         "drifted by more than 1 sample between repeats\n")
        return 4
    return 0


def _print_summary(analyses: list[CellAnalysis]) -> None:
    per_path_residuals: dict[str, list[float]] = {
        "direct": [], "surface": [], "bottom": []
    }
    for a in analyses:
        for name in per_path_residuals:
            if name not in a.analytical:
                continue
            meas = a.measured.get(name, float("nan"))
            ana  = a.analytical[name]
            if not math.isnan(meas):
                per_path_residuals[name].append(abs(meas - ana))
    sys.stderr.write("[exp1] residuals (|measured − analytical|, samples):\n")
    for name, vals in per_path_residuals.items():
        if not vals:
            continue
        arr = np.asarray(vals)
        sys.stderr.write(
            f"  {name:7s}  n={arr.size:3d}  "
            f"mean={arr.mean():.2f}  "
            f"p99={np.percentile(arr, 99):.2f}  "
            f"max={arr.max():.2f}\n")


if __name__ == "__main__":
    sys.exit(main())
