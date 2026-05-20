#!/usr/bin/env python3
"""
Verify that the host's Doppler resampler shifted the modem's TX spectrum by
the expected ratio (1 + v/c) before re-injecting it as RX. Compares dominant
FSK tone peaks across the pre-channel TX WAV and the post-channel RX WAV.

Usage:
  python3 scripts/verify_doppler_shift.py \
      --tx logs/OA-2-1_tx.wav \
      --rx logs/OA-2-1_rx_001.wav \
      --velocity 10.0 \
      [--sound-speed 1500.0] \
      [--tol 0.001]

Exit code 0 on PASS, 1 on FAIL.
"""
import argparse
import sys

import numpy as np
from scipy.io import wavfile
from scipy.signal import welch, find_peaks


# FSK band of the OpenAquatix modem (cfg_defaults.h: f0=29 kHz, f1=30 kHz).
# Widened a little to leave headroom for Doppler shifts up to ~5%.
BAND_LOW_HZ = 25_000.0
BAND_HIGH_HZ = 40_000.0


def load_wav(path):
    rate, data = wavfile.read(path)
    if data.ndim > 1:
        data = data[:, 0]
    return rate, data.astype(np.float64) / 32768.0


def two_strongest_tone_peaks(rate, x):
    """Return the two strongest spectral peaks (Hz) in [BAND_LOW, BAND_HIGH].

    Uses Welch PSD (segments ~50 ms, Hann) for variance reduction over a
    multi-second capture, then `find_peaks` with a minimum spacing of 500 Hz
    so the two FSK tones (1 kHz apart at nominal) don't collapse into one.
    """
    nperseg = min(len(x), max(2048, int(rate * 0.05)))
    f, psd = welch(x - x.mean(), fs=rate, window='hann',
                   nperseg=nperseg, noverlap=nperseg // 2)

    band_mask = (f >= BAND_LOW_HZ) & (f <= BAND_HIGH_HZ)
    f_band = f[band_mask]
    psd_band = psd[band_mask]
    if f_band.size == 0:
        raise RuntimeError(f'no PSD bins in [{BAND_LOW_HZ}, {BAND_HIGH_HZ}] Hz '
                           f'(rate={rate})')

    bin_hz = f_band[1] - f_band[0]
    distance_bins = max(1, int(round(500.0 / bin_hz)))

    peak_idx, props = find_peaks(psd_band, distance=distance_bins,
                                  prominence=psd_band.max() * 1e-3)
    if peak_idx.size < 2:
        # Fall back to the two largest single bins — may happen on very short
        # captures where Welch averaging didn't smooth out enough.
        order = np.argsort(psd_band)[::-1]
        peaks_hz = sorted(f_band[order[:2]].tolist())
        return peaks_hz, psd_band[order[:2]].tolist()

    # Sort the found peaks by power, take top two, then return them in
    # ascending-frequency order so f0 (lower) maps to f0.
    by_power = peak_idx[np.argsort(psd_band[peak_idx])[::-1][:2]]
    sorted_by_freq = by_power[np.argsort(f_band[by_power])]
    peaks_hz = f_band[sorted_by_freq].tolist()
    powers = psd_band[sorted_by_freq].tolist()
    return peaks_hz, powers


def main():
    ap = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tx', required=True, help='Pre-channel TX WAV (modem → host)')
    ap.add_argument('--rx', required=True, help='Post-channel RX WAV (host → modem)')
    ap.add_argument('--velocity', type=float, required=True,
                    help='Configured velocity_radial_m_s in the scenario')
    ap.add_argument('--sound-speed', type=float, default=1500.0,
                    help='Sound speed (m/s) used in the scenario [1500]')
    ap.add_argument('--tol', type=float, default=0.001,
                    help='Allowed |ratio − expected| / expected [0.001]')
    args = ap.parse_args()

    expected_ratio = 1.0 + args.velocity / args.sound_speed

    rate_tx, tx = load_wav(args.tx)
    rate_rx, rx = load_wav(args.rx)
    if rate_tx != rate_rx:
        print(f'ERROR: sample rates differ (tx={rate_tx}, rx={rate_rx})',
              file=sys.stderr)
        return 1

    tx_peaks, _ = two_strongest_tone_peaks(rate_tx, tx)
    rx_peaks, _ = two_strongest_tone_peaks(rate_rx, rx)

    # Pair each TX peak with the nearest RX peak within ±5%.
    pairs = []
    for f_tx in tx_peaks:
        best = min(rx_peaks, key=lambda f_rx: abs(f_rx / f_tx - expected_ratio))
        if abs(best / f_tx - 1.0) > 0.05:
            print(f'ERROR: no RX peak within 5% of TX peak {f_tx/1e3:.3f} kHz '
                  f'(rx peaks: {[f"{p/1e3:.3f}" for p in rx_peaks]})',
                  file=sys.stderr)
            return 1
        pairs.append((f_tx, best))

    print(f'Configured velocity:   {args.velocity:+.3f} m/s')
    print(f'Sound speed:           {args.sound_speed:.1f} m/s')
    print(f'Expected ratio:        {expected_ratio:.6f}')
    print(f'Tolerance:             ±{args.tol*100:.3f}% of expected')
    print()
    print(f'{"TX peak (kHz)":>14} {"RX peak (kHz)":>14} {"Ratio":>10} '
          f'{"Δ vs exp":>11} {"Verdict":>8}')

    all_pass = True
    for f_tx, f_rx in pairs:
        ratio = f_rx / f_tx
        rel_dev = (ratio - expected_ratio) / expected_ratio
        ok = abs(rel_dev) <= args.tol
        all_pass = all_pass and ok
        print(f'{f_tx/1e3:>14.4f} {f_rx/1e3:>14.4f} {ratio:>10.6f} '
              f'{rel_dev*100:>+10.4f}% {"PASS" if ok else "FAIL":>8}')

    print()
    print('OVERALL:', 'PASS' if all_pass else 'FAIL')
    return 0 if all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
