"""Quasi-static tap extraction from a measured TVIR.

One window of a Watermark-style record reduces to a frozen tap set
{delay, amplitude, Doppler} estimated from the window's scattering
function (FFT of ``h[time, delay]`` along record time). A tap with a
constant Doppler shift ``nu`` at the measurement carrier ``fc`` is
exactly a linear delay ramp ``d tau/dt = -nu/fc``, so the emitted
``.octt`` holds linear per-tap delay ramps with constant amplitudes —
the replay renderer plays it back with no special casing, and any modem
carrier sees the correct shift (``fc_modem * d tau/dt``).

Compared to the track-based converter in :mod:`lib.watermark`, this path
has no association step to defend: each delay-Doppler peak *is* a tap.
The cost is that all time variation inside the window (fading, births,
deaths) is discarded.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .tap_trajectory import MAX_DELAY_S, MAX_TAPS
from .watermark import (GUARD_DELAY_S, SOUND_SPEED_M_S, ConversionResult,
                        TVIRData)

# Below this many snapshots the Doppler axis is too coarse to mean
# anything (a Hann mainlobe spans 4 bins).
MIN_WINDOW_FRAMES = 8


@dataclass
class ScatteringFunction:
    """Windowed delay-Doppler amplitude map of one TVIR segment."""
    amplitude: np.ndarray    # [doppler_bins, delay_bins] linear amplitude
    doppler_hz: np.ndarray   # [doppler_bins] fftshift-ordered axis
    delay_s: np.ndarray      # [delay_bins]
    resolution_hz: float     # Doppler bin spacing (Hann mainlobe = 4 bins)
    window_start_s: float
    window_duration_s: float


@dataclass
class QuasiStaticTaps:
    """A frozen tap set extracted from one record window.

    Delays sit on the measured delay axis (no renderer anchoring);
    ``doppler_hz`` is the residual per-tap Doppler at ``fc_meas_hz`` —
    the mean Doppler Watermark removed is carried separately in
    ``v0_m_s`` and reinstalled at trajectory emission.
    """
    delays_s: np.ndarray            # [K] seconds, ascending
    doppler_hz: np.ndarray          # [K]
    amplitudes: np.ndarray          # [K] linear, peak == 1.0
    fc_meas_hz: float
    v0_m_s: float
    window_start_s: float
    window_duration_s: float
    doppler_resolution_hz: float
    captured_energy_fraction: float
    normalization_db: float
    # Peak-over-median of the scattering function. A real channel shows
    # 30+ dB; a structureless window (noise, sounding not yet started)
    # sits near 10 dB — its "taps" are speckle maxima, not arrivals.
    dynamic_range_db: float = float("nan")

    @property
    def tap_count(self) -> int:
        return len(self.delays_s)


def scattering_function(tvir: TVIRData,
                        *,
                        start_s: float = 0.0,
                        duration_s: float | None = None,
                        ) -> ScatteringFunction:
    """Hann-windowed FFT of ``h`` along record time, normalised so a
    constant-amplitude path reads its amplitude at its Doppler line."""
    i0 = int(round(start_s * tvir.fs_t))
    n_total = tvir.h.shape[0]
    n = (n_total - i0 if duration_s is None
         else int(round(duration_s * tvir.fs_t)))
    if i0 < 0 or i0 + n > n_total:
        raise ValueError(f"window [{start_s}, {start_s}+{duration_s}] s "
                         f"exceeds the {n_total / tvir.fs_t:.1f} s record")
    if n < MIN_WINDOW_FRAMES:
        raise ValueError(f"window has {n} snapshots; need at least "
                         f"{MIN_WINDOW_FRAMES} for a usable Doppler axis")

    segment = tvir.h[i0:i0 + n]
    w = np.hanning(n)
    spectrum = np.fft.fftshift(
        np.fft.fft(segment * w[:, None], axis=0), axes=0) / w.sum()
    return ScatteringFunction(
        amplitude=np.abs(spectrum),
        doppler_hz=np.fft.fftshift(np.fft.fftfreq(n, d=1.0 / tvir.fs_t)),
        delay_s=np.arange(tvir.h.shape[1]) / tvir.fs_tau,
        resolution_hz=tvir.fs_t / n,
        window_start_s=i0 / tvir.fs_t,
        window_duration_s=n / tvir.fs_t,
    )


# ---------------------------------------------------------------------------
# 2-D peak picking
# ---------------------------------------------------------------------------

def _log_parabolic(y_m1: float, y_0: float, y_p1: float) -> tuple[float, float]:
    """Sub-bin peak offset and value gain from a 3-point parabola on the
    log magnitude (log-domain fits undo most window scalloping)."""
    eps = 1e-30
    l_m1, l_0, l_p1 = (np.log(y_m1 + eps), np.log(y_0 + eps),
                       np.log(y_p1 + eps))
    denom = l_m1 - 2.0 * l_0 + l_p1
    if denom >= 0.0:
        return 0.0, 1.0
    off = float(np.clip(0.5 * (l_m1 - l_p1) / denom, -0.5, 0.5))
    gain = float(np.exp(-0.25 * (l_m1 - l_p1) * off))
    return off, gain


def extract_quasi_static(tvir: TVIRData,
                         *,
                         start_s: float = 0.0,
                         duration_s: float | None = None,
                         max_taps: int = MAX_TAPS,
                         prominence_db: float = -30.0,
                         min_separation_bins: int = 2,
                         min_separation_doppler_bins: int = 2,
                         ) -> QuasiStaticTaps:
    """Peak-pick the windowed scattering function into a frozen tap set.

    A candidate is suppressed only when it is close to a kept peak in
    BOTH axes, so two paths at the same delay with distinct Doppler
    survive as distinct taps (which no single |h| snapshot can resolve).
    """
    sf = scattering_function(tvir, start_s=start_s, duration_s=duration_s)
    amp = sf.amplitude
    threshold = float(amp.max()) * 10.0 ** (prominence_db / 20.0)

    n_dopp, n_delay = amp.shape
    interior = amp[1:-1, 1:-1]
    is_peak = ((interior >= amp[:-2, 1:-1]) & (interior > amp[2:, 1:-1]) &
               (interior >= amp[1:-1, :-2]) & (interior > amp[1:-1, 2:]) &
               (interior >= threshold))
    cand_i, cand_j = np.nonzero(is_peak)
    cand_i += 1
    cand_j += 1
    order = np.argsort(amp[cand_i, cand_j])[::-1]

    kept: list[tuple[int, int]] = []
    for idx in order:
        i, j = int(cand_i[idx]), int(cand_j[idx])
        if any(abs(i - ki) < min_separation_doppler_bins
               and abs(j - kj) < min_separation_bins
               for ki, kj in kept):
            continue
        kept.append((i, j))
    if not kept:
        raise ValueError("no delay-Doppler peaks above the prominence "
                         "threshold — check the window or prominence_db")

    total_power = sum(float(amp[i, j]) ** 2 for i, j in kept)
    kept = kept[:max_taps]
    captured = (sum(float(amp[i, j]) ** 2 for i, j in kept) / total_power
                if total_power > 0.0 else 0.0)

    bin_s = 1.0 / tvir.fs_tau
    delays = np.empty(len(kept))
    dopplers = np.empty(len(kept))
    amps = np.empty(len(kept))
    for k, (i, j) in enumerate(kept):
        off_i, gain_i = _log_parabolic(amp[i - 1, j], amp[i, j], amp[i + 1, j])
        off_j, gain_j = _log_parabolic(amp[i, j - 1], amp[i, j], amp[i, j + 1])
        dopplers[k] = sf.doppler_hz[i] + off_i * sf.resolution_hz
        delays[k] = (j + off_j) * bin_s
        amps[k] = amp[i, j] * gain_i * gain_j

    ascending = np.argsort(delays)
    delays, dopplers, amps = (delays[ascending], dopplers[ascending],
                              amps[ascending])

    peak = float(amps.max())
    return QuasiStaticTaps(
        delays_s=delays,
        doppler_hz=dopplers,
        amplitudes=amps / peak,
        fc_meas_hz=tvir.fc,
        v0_m_s=tvir.v0,
        window_start_s=sf.window_start_s,
        window_duration_s=sf.window_duration_s,
        doppler_resolution_hz=sf.resolution_hz,
        captured_energy_fraction=float(captured),
        normalization_db=20.0 * float(np.log10(peak)),
        dynamic_range_db=20.0 * float(
            np.log10(amp.max() / max(np.median(amp), 1e-30))),
    )


# ---------------------------------------------------------------------------
# Trajectory emission
# ---------------------------------------------------------------------------

def quasi_static_to_trajectories(taps: QuasiStaticTaps,
                                 *,
                                 record_duration_s: float = 30.0,
                                 dt_s: float = 1.0,
                                 reinstall_v0: bool = True,
                                 sound_speed_m_s: float = SOUND_SPEED_M_S,
                                 guard_delay_s: float = GUARD_DELAY_S,
                                 ) -> ConversionResult:
    """Render the frozen tap set as linear delay ramps on a uniform grid.

    Interior Catmull-Rom segments reproduce a linear track exactly, and
    the clamped end segments cannot overshoot a monotone ramp, so the
    guard-floor anchor is safe by construction. Keep messages inside
    ``[dt_s, record_duration_s - dt_s]`` (set the scenario's ``offset_s``
    to ``dt_s``) so playback only touches the exact interior segments.
    """
    slopes = -taps.doppler_hz / taps.fc_meas_hz
    if reinstall_v0 and taps.v0_m_s != 0.0:
        slopes = slopes + taps.v0_m_s / sound_speed_m_s

    frames = int(round(record_duration_s / dt_s)) + 1
    if frames < 2:
        raise ValueError("record_duration_s must cover at least one frame "
                         "interval")
    t = np.arange(frames) * dt_s
    delays = taps.delays_s[None, :] + slopes[None, :] * t[:, None]

    anchor_offset = guard_delay_s - float(delays.min())
    delays = delays + anchor_offset

    if float(delays.max()) > MAX_DELAY_S:
        raise ValueError(
            f"delay span plus Doppler drift reaches {delays.max():.4f} s "
            f"over the {record_duration_s:.1f} s record, exceeding the "
            f"{MAX_DELAY_S} s channel limit — shorten record_duration_s")

    amplitudes = np.tile(taps.amplitudes.astype(np.float32), (frames, 1))
    return ConversionResult(
        delays=delays,
        amplitudes=amplitudes,
        dt_s=dt_s,
        fc_meas_hz=taps.fc_meas_hz,
        normalization_db=taps.normalization_db,
        captured_energy_fraction=taps.captured_energy_fraction,
        delay_anchor_offset_s=float(anchor_offset),
    )


# ---------------------------------------------------------------------------
# QA figure
# ---------------------------------------------------------------------------

def render_quasi_static_qa(tvir: TVIRData, taps: QuasiStaticTaps,
                           savepath: str | Path,
                           *,
                           floor_db: float = -40.0) -> Path:
    """Two-panel QA figure: the windowed scattering function with the
    extracted peaks circled (top), and the tap amplitude/Doppler summary
    (bottom). Extraction failures show up as circles off the energy
    ridges or ridges with no circle.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sf = scattering_function(tvir, start_s=taps.window_start_s,
                             duration_s=taps.window_duration_s)
    amp_db = 20.0 * np.log10(sf.amplitude + 1e-12)
    amp_db -= amp_db.max()
    delay_ms = sf.delay_s * 1000.0

    fig, (ax_sf, ax_taps) = plt.subplots(
        2, 1, figsize=(6.0, 6.0),
        gridspec_kw={"height_ratios": [2.2, 1.0]})

    im = ax_sf.imshow(
        amp_db, aspect="auto", origin="lower", cmap="viridis",
        vmin=floor_db, vmax=0.0,
        extent=[delay_ms[0], delay_ms[-1],
                sf.doppler_hz[0], sf.doppler_hz[-1]])
    ax_sf.scatter(taps.delays_s * 1000.0, taps.doppler_hz,
                  s=40, facecolors="none", edgecolors="red", linewidths=1.0)
    ax_sf.set_xlabel("Delay (ms)")
    ax_sf.set_ylabel("Doppler (Hz)")
    ax_sf.set_title(
        f"{taps.tap_count} taps, window "
        f"[{taps.window_start_s:.1f}, "
        f"{taps.window_start_s + taps.window_duration_s:.1f}] s, "
        f"captured {taps.captured_energy_fraction:.2f}, "
        f"norm {taps.normalization_db:+.1f} dB, "
        f"res {taps.doppler_resolution_hz:.2f} Hz", fontsize=9)
    fig.colorbar(im, ax=ax_sf, label="|S| (dB)")

    with np.errstate(divide="ignore"):
        tap_db = 20.0 * np.log10(taps.amplitudes)
    markers, stems, _ = ax_taps.stem(taps.delays_s * 1000.0, tap_db,
                                     bottom=floor_db)
    plt.setp(stems, linewidth=1.0)
    plt.setp(markers, markersize=4)
    for d_ms, a_db, nu in zip(taps.delays_s * 1000.0, tap_db,
                              taps.doppler_hz):
        ax_taps.annotate(f"{nu:+.2f} Hz", (d_ms, a_db),
                         textcoords="offset points", xytext=(3, 3),
                         fontsize=7)
    ax_taps.set_xlabel("Delay (ms)")
    ax_taps.set_ylabel("Tap amplitude (dB)")
    ax_taps.set_ylim(floor_db, 3.0)

    out = Path(savepath)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return out
