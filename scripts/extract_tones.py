#!/usr/bin/env python3
"""
Extract the dominant tone per 10 ms symbol from each WAV file and compare
hop sequences.  Useful for confirming whether two FH-BFSK recordings are
the same message or different messages.

Usage:
  python3 scripts/extract_tones.py logs/OA-2-1_tx.wav logs/OA-2-1_rx_004.wav
"""
import sys
import numpy as np
from scipy.io import wavfile

SYMBOL_MS = 10.0

def load(path):
    rate, data = wavfile.read(path)
    if data.ndim > 1:
        data = data[:, 0]
    return rate, data.astype(np.float32) / 32768.0

def trim_silence(x, thresh=0.005):
    mask = np.abs(x) > thresh
    if not mask.any():
        return x
    i0 = int(np.argmax(mask))
    i1 = len(x) - int(np.argmax(mask[::-1]))
    return x[i0:i1]

def hop_sequence(x, rate, symbol_ms=SYMBOL_MS, fmin=25e3, fmax=35e3):
    n_per_sym = int(round(symbol_ms * 1e-3 * rate))
    nsym = len(x) // n_per_sym
    freqs = np.fft.rfftfreq(n_per_sym, 1.0 / rate)
    band = (freqs >= fmin) & (freqs <= fmax)
    hops = []
    for i in range(nsym):
        seg = x[i * n_per_sym:(i + 1) * n_per_sym]
        if np.sqrt(np.mean(seg * seg)) < 0.002:
            hops.append(None)  # silence
            continue
        spec = np.abs(np.fft.rfft(seg - seg.mean()))
        k = int(np.argmax(spec[band]))
        hops.append(float(freqs[band][k]))
    return hops

def main(paths):
    rate_a, a = load(paths[0])
    rate_b, b = load(paths[1])
    a = trim_silence(a)
    b = trim_silence(b)
    print(f'{paths[0]}: {len(a)} samples ({len(a)/rate_a*1e3:.1f} ms)')
    print(f'{paths[1]}: {len(b)} samples ({len(b)/rate_b*1e3:.1f} ms)')

    hops_a = hop_sequence(a, rate_a)
    hops_b = hop_sequence(b, rate_b)
    n = min(len(hops_a), len(hops_b))
    print(f'\nFirst {min(n, 40)} symbols (kHz):')
    print('sym  TX_file     RX_file    delta(kHz)')
    matches = 0
    total = 0
    for i in range(min(n, 40)):
        a_f = hops_a[i]
        b_f = hops_b[i]
        if a_f is None or b_f is None:
            print(f'{i:3d}  --         --         --')
            continue
        d = (b_f - a_f) / 1e3
        same = abs(d) < 0.15    # within 150 Hz → same tone
        mark = '  =' if same else '  X'
        if same: matches += 1
        total += 1
        print(f'{i:3d}  {a_f/1e3:6.2f}     {b_f/1e3:6.2f}   {d:+6.2f}   {mark}')

    if total > 0:
        print(f'\nMatching symbols: {matches}/{total}  ({100.0*matches/total:.0f}%)')
        print('(>90% match = same message; <30% = different messages)')

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    main(sys.argv[1:])
