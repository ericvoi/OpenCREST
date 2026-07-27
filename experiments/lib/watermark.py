"""Watermark-style TVIR → sparse tap-trajectory conversion.

The pipeline reduces a measured time-varying impulse response (complex
baseband ``h[time, delay]`` plus its grid metadata, as shipped by the
Watermark benchmark) to per-tap delay/amplitude trajectories the C++
replay mode can render at real passband:

  1. Per snapshot, pick the dominant |h| peaks (sub-bin delay via
     parabolic interpolation).
  2. Associate peaks across snapshots into tap tracks (nearest-neighbour
     with a delay gate, full-file hindsight ranking).
  3. Convert each track's unwrapped baseband phase into fine delay
     (dphi/dt = -2*pi*fc*dtau/dt), so Doppler transfers to any carrier.
  4. Reinstall the removed mean Doppler V0 as a linear delay ramp.
  5. Resolve birth/death into amplitude ramps (zero outside the track's
     lifetime; delay holds its edge value so interpolation stays sane).
  6. Smooth, re-anchor above the interpolation guard floor, and
     peak-normalise amplitudes.

Everything non-causal (association, smoothing, ranking) happens here,
offline; the C++ side only interpolates the result.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .tap_trajectory import MAX_TAPS, catmull_rom_uniform

SOUND_SPEED_M_S = 1500.0

# Interpolation guard floor for the delay track (seconds). Catmull-Rom can
# overshoot below the raw sample minimum; the converter re-anchors the
# global minimum here and then verifies the densely-interpolated track
# stays >= 0.
GUARD_DELAY_S = 2e-5


@dataclass
class TVIRData:
    """A measured TVIR: complex baseband h over a uniform (time, delay) grid."""
    h: np.ndarray        # complex128 [snapshots, delay_bins]
    fs_t: float          # snapshot rate, Hz
    fs_tau: float        # delay-axis sampling rate, Hz
    fc: float            # measurement centre frequency, Hz
    v0: float = 0.0      # mean Doppler removed before estimation, m/s


@dataclass
class TapTrack:
    """One associated arrival across snapshots (indices into the TVIR grid)."""
    first_frame: int
    delays_s: list[float] = field(default_factory=list)      # coarse+sub-bin
    amplitudes: list[float] = field(default_factory=list)    # |h| at peak
    phases: list[float] = field(default_factory=list)        # raw angle(h)

    @property
    def last_frame(self) -> int:
        return self.first_frame + len(self.delays_s) - 1

    @property
    def energy(self) -> float:
        return float(np.sum(np.square(self.amplitudes)))


def load_tvir(path: str | Path) -> TVIRData:
    """Load a Watermark channel file (``h``, ``fs_t``, ``fs_tau``, ``fc``,
    ``V0``). Handles both classic .mat and v7.3 (HDF5) layouts."""
    p = Path(path)
    try:
        from scipy.io import loadmat
        mat = loadmat(str(p))
        h = np.asarray(mat["h"])
        fs_t = float(np.squeeze(mat["fs_t"]))
        fs_tau = float(np.squeeze(mat["fs_tau"]))
        fc = float(np.squeeze(mat["fc"]))
        v0 = float(np.squeeze(mat["V0"])) if "V0" in mat else 0.0
    except NotImplementedError:
        import h5py
        with h5py.File(str(p), "r") as f:
            raw = f["h"][()]
            # v7.3 stores complex as a structured array and transposes.
            if raw.dtype.names and {"real", "imag"} <= set(raw.dtype.names):
                h = (raw["real"] + 1j * raw["imag"]).T
            else:
                h = np.asarray(raw).T
            fs_t = float(np.squeeze(f["fs_t"][()]))
            fs_tau = float(np.squeeze(f["fs_tau"][()]))
            fc = float(np.squeeze(f["fc"][()]))
            v0 = float(np.squeeze(f["V0"][()])) if "V0" in f else 0.0
    if h.ndim != 2:
        raise ValueError(f"{p}: h must be 2-D [time, delay], got {h.shape}")
    return TVIRData(h=np.asarray(h, dtype=np.complex128),
                    fs_t=fs_t, fs_tau=fs_tau, fc=fc, v0=v0)


# ---------------------------------------------------------------------------
# Peak picking + track association
# ---------------------------------------------------------------------------

def _parabolic_offset(y_m1: float, y_0: float, y_p1: float) -> float:
    """Sub-bin offset of a peak from a 3-point parabola fit, in bins."""
    denom = y_m1 - 2.0 * y_0 + y_p1
    if denom >= 0.0:
        return 0.0
    return float(np.clip(0.5 * (y_m1 - y_p1) / denom, -0.5, 0.5))


def _snapshot_peaks(mag: np.ndarray, threshold: float,
                    min_separation_bins: int) -> list[int]:
    """Local maxima above threshold, strongest-first, greedily enforcing a
    minimum bin separation."""
    candidates = [
        b for b in range(1, len(mag) - 1)
        if mag[b] >= threshold and mag[b] >= mag[b - 1] and mag[b] > mag[b + 1]
    ]
    candidates.sort(key=lambda b: mag[b], reverse=True)
    kept: list[int] = []
    for b in candidates:
        if all(abs(b - k) >= min_separation_bins for k in kept):
            kept.append(b)
    return kept


def extract_tracks(tvir: TVIRData,
                   *,
                   prominence_db: float = -30.0,
                   min_separation_bins: int = 2,
                   gate_bins: float = 3.0,
                   min_track_frames: int = 3,
                   ) -> list[TapTrack]:
    """Associate per-snapshot |h| peaks into tap tracks.

    ``prominence_db``: pick threshold relative to the global |h| maximum.
    ``gate_bins``: max per-snapshot delay movement for track continuation.
    ``min_track_frames``: shorter tracks are discarded as clutter.
    """
    mag = np.abs(tvir.h)
    threshold = float(mag.max()) * 10.0 ** (prominence_db / 20.0)
    bin_s = 1.0 / tvir.fs_tau
    gate_s = gate_bins * bin_s

    active: list[TapTrack] = []
    finished: list[TapTrack] = []

    for i in range(tvir.h.shape[0]):
        row = mag[i]
        peaks = _snapshot_peaks(row, threshold, min_separation_bins)
        entries = []
        for b in peaks:
            off = _parabolic_offset(row[b - 1], row[b], row[b + 1])
            entries.append({
                "delay_s": (b + off) * bin_s,
                "amp": float(row[b]),
                "phase": float(np.angle(tvir.h[i, b])),
            })

        # Greedy nearest-neighbour: strongest tracks pick first.
        unmatched = list(entries)
        still_active: list[TapTrack] = []
        for tr in sorted(active, key=lambda t: t.energy, reverse=True):
            best = None
            best_dist = gate_s
            for e in unmatched:
                d = abs(e["delay_s"] - tr.delays_s[-1])
                if d < best_dist:
                    best, best_dist = e, d
            if best is not None:
                unmatched.remove(best)
                tr.delays_s.append(best["delay_s"])
                tr.amplitudes.append(best["amp"])
                tr.phases.append(best["phase"])
                still_active.append(tr)
            else:
                finished.append(tr)
        for e in unmatched:
            tr = TapTrack(first_frame=i)
            tr.delays_s.append(e["delay_s"])
            tr.amplitudes.append(e["amp"])
            tr.phases.append(e["phase"])
            still_active.append(tr)
        active = still_active

    finished.extend(active)
    return [t for t in finished if len(t.delays_s) >= min_track_frames]


# ---------------------------------------------------------------------------
# Tracks → uniform trajectories
# ---------------------------------------------------------------------------

@dataclass
class ConversionResult:
    delays: np.ndarray          # [frames, taps] float64, seconds
    amplitudes: np.ndarray      # [frames, taps] float32, peak-normalised
    dt_s: float
    fc_meas_hz: float
    normalization_db: float     # dB removed by peak normalisation
    captured_energy_fraction: float
    # Constant added to every delay by guard-floor re-anchoring; subtract
    # it (and the V0 ramp) to land back on the measured delay axis.
    delay_anchor_offset_s: float = 0.0


def _savgol_or_boxcar(x: np.ndarray, window: int) -> np.ndarray:
    if window < 3 or len(x) < window:
        return x
    if window % 2 == 0:
        window += 1
    try:
        from scipy.signal import savgol_filter
        return savgol_filter(x, window_length=window, polyorder=2)
    except ImportError:
        kernel = np.ones(window) / window
        pad = window // 2
        padded = np.pad(x, pad, mode="edge")
        return np.convolve(padded, kernel, mode="valid")


def tracks_to_trajectories(tracks: list[TapTrack],
                           tvir: TVIRData,
                           *,
                           max_taps: int = MAX_TAPS,
                           smooth_frames: int = 5,
                           sound_speed_m_s: float = SOUND_SPEED_M_S,
                           guard_delay_s: float = GUARD_DELAY_S,
                           ) -> ConversionResult:
    """Convert tracks to the uniform per-tap grids the ``.octt`` writer
    expects. Raises ValueError if no track survives."""
    if not tracks:
        raise ValueError("no tap tracks to convert")

    kept = sorted(tracks, key=lambda t: t.energy, reverse=True)[:max_taps]
    total_energy = sum(t.energy for t in tracks)
    captured = sum(t.energy for t in kept) / total_energy if total_energy else 0.0

    frames = tvir.h.shape[0]
    dt = 1.0 / tvir.fs_t
    n_taps = len(kept)
    delays = np.zeros((frames, n_taps), dtype=np.float64)
    amps = np.zeros((frames, n_taps), dtype=np.float32)

    for k, tr in enumerate(kept):
        span = slice(tr.first_frame, tr.last_frame + 1)
        coarse = np.asarray(tr.delays_s)

        # Phase -> fine delay: dphi/dt = -2*pi*fc*dtau/dt. The constant
        # offset is anchored so the fine track starts at the coarse start
        # (absolute delay beyond bin resolution is not recoverable). The
        # phase carries all in-coverage motion including bin-scale drift
        # and reflection phase flips, so it is used as the sole delay
        # source after anchoring; unwrap cycle slips are bounded by the
        # smoothing window.
        phase = np.unwrap(np.asarray(tr.phases))
        fine = coarse[0] - (phase - phase[0]) / (2.0 * np.pi * tvir.fc)
        delay_track = _savgol_or_boxcar(fine, smooth_frames)

        delays[span, k] = delay_track
        # Hold edge delay outside the lifetime; amplitude stays 0 there,
        # which the renderer treats as the birth/death ramp endpoints.
        delays[:tr.first_frame, k] = delay_track[0]
        delays[tr.last_frame + 1:, k] = delay_track[-1]
        amps[span, k] = _savgol_or_boxcar(
            np.asarray(tr.amplitudes), smooth_frames).clip(min=0.0)

    # Reinstall the removed mean Doppler as a linear delay ramp.
    if tvir.v0 != 0.0:
        t = np.arange(frames) * dt
        delays += (tvir.v0 / sound_speed_m_s) * t[:, None]

    # Re-anchor the global minimum onto the guard floor, then verify the
    # interpolated track can't undershoot below zero.
    anchor_offset = guard_delay_s - delays.min()
    delays += anchor_offset
    interp_min = _dense_interpolated_min(delays, dt)
    if interp_min < 0.0:
        anchor_offset += guard_delay_s - interp_min
        delays += guard_delay_s - interp_min

    peak = float(amps.max())
    if peak <= 0.0:
        raise ValueError("all tap amplitudes are zero")
    amps /= peak

    return ConversionResult(
        delays=delays,
        amplitudes=amps,
        dt_s=dt,
        fc_meas_hz=tvir.fc,
        normalization_db=20.0 * float(np.log10(peak)),
        captured_energy_fraction=float(captured),
        delay_anchor_offset_s=float(anchor_offset),
    )


def _dense_interpolated_min(delays: np.ndarray, dt: float,
                            oversample: int = 16) -> float:
    """Minimum of the Catmull-Rom-interpolated delay tracks, densely
    sampled — the same interpolation the C++ renderer applies."""
    frames, taps = delays.shape
    duration = (frames - 1) * dt
    ts = np.linspace(0.0, duration, (frames - 1) * oversample + 1)
    lowest = np.inf
    for k in range(taps):
        track = delays[:, k]
        for t in ts:
            lowest = min(lowest, catmull_rom_uniform(track, dt, t))
    return float(lowest)


def render_conversion_qa(tvir: TVIRData, result: ConversionResult,
                         savepath: str | Path,
                         *,
                         floor_db: float = -40.0,
                         sound_speed_m_s: float = SOUND_SPEED_M_S) -> Path:
    """Two-panel QA figure: extracted tap trajectories overlaid on the
    |h(t,tau)| waterfall (top), and per-tap amplitude tracks (bottom).

    Inspect this before trusting a conversion — track-association failures
    show up as trajectories jumping between arrivals or missing energy
    ridges entirely.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frames = tvir.h.shape[0]
    t = np.arange(frames) / tvir.fs_t
    delay_ms = np.arange(tvir.h.shape[1]) / tvir.fs_tau * 1000.0

    mag_db = 20.0 * np.log10(np.abs(tvir.h) + 1e-12)
    mag_db -= mag_db.max()

    # Land the stored (anchored, V0-reinstalled) delays back on the
    # measured axis.
    overlay = (result.delays
               - result.delay_anchor_offset_s
               - (tvir.v0 / sound_speed_m_s) * t[:, None]) * 1000.0

    fig, (ax_wf, ax_amp) = plt.subplots(
        2, 1, figsize=(6.0, 6.0), sharex=False,
        gridspec_kw={"height_ratios": [2.2, 1.0]})

    im = ax_wf.imshow(
        mag_db, aspect="auto", origin="lower", cmap="viridis",
        vmin=floor_db, vmax=0.0,
        extent=[delay_ms[0], delay_ms[-1], t[0], t[-1]])
    for k in range(result.delays.shape[1]):
        alive = result.amplitudes[:, k] > 0.0
        ax_wf.plot(np.where(alive, overlay[:, k], np.nan), t,
                   lw=0.8, color="red", alpha=0.8)
    ax_wf.set_xlabel("Delay (ms)")
    ax_wf.set_ylabel("Record time (s)")
    ax_wf.set_title(
        f"{result.delays.shape[1]} taps, "
        f"captured energy {result.captured_energy_fraction:.2f}, "
        f"norm {result.normalization_db:+.1f} dB", fontsize=9)
    fig.colorbar(im, ax=ax_wf, label="|h| (dB)")

    with np.errstate(divide="ignore"):
        amp_db = 20.0 * np.log10(result.amplitudes)
    for k in range(result.amplitudes.shape[1]):
        ax_amp.plot(t, amp_db[:, k], lw=0.8)
    ax_amp.set_xlabel("Record time (s)")
    ax_amp.set_ylabel("Tap amplitude (dB)")
    ax_amp.set_ylim(floor_db, 2.0)

    out = Path(savepath)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return out


def convert(tvir: TVIRData, **kwargs) -> ConversionResult:
    """extract_tracks + tracks_to_trajectories with shared defaults."""
    extract_keys = {"prominence_db", "min_separation_bins", "gate_bins",
                    "min_track_frames"}
    extract_kwargs = {k: v for k, v in kwargs.items() if k in extract_keys}
    traj_kwargs = {k: v for k, v in kwargs.items() if k not in extract_keys}
    tracks = extract_tracks(tvir, **extract_kwargs)
    return tracks_to_trajectories(tracks, tvir, **traj_kwargs)
