#!/usr/bin/env python3
"""
Compare two WAV files (typically tx vs rx loopback) in time and frequency.

Usage:
  python3 scripts/analyze_wav.py logs/OA-2-1_tx.wav logs/OA-2-1_rx_004.wav
  python3 scripts/analyze_wav.py logs/OA-2-1_tx.wav           # single file

Output PNG is written next to the first input file. Stats print to stdout.
"""
import sys
import os
import wave
import numpy as np
from scipy.signal import spectrogram, correlate
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(path):
    # Tolerant WAV reader: the simulator's streaming writer leaves RIFF/data
    # chunk sizes at 0 if it didn't close gracefully (Ctrl-C). Parse fmt
    # manually and read samples to EOF when the declared size is bogus.
    import struct
    with open(path, 'rb') as f:
        if f.read(4) != b'RIFF':
            raise ValueError(f'{path}: not a RIFF file')
        f.read(4)                                # riff size (ignored)
        if f.read(4) != b'WAVE':
            raise ValueError(f'{path}: not a WAVE file')
        rate = sampwidth = nchannels = None
        data_offset = data_size = None
        while True:
            hdr = f.read(8)
            if len(hdr) < 8:
                break
            chunk_id, chunk_size = struct.unpack('<4sI', hdr)
            if chunk_id == b'fmt ':
                fmt = f.read(chunk_size)
                _, nchannels, rate, _, _, bits = struct.unpack('<HHIIHH', fmt[:16])
                sampwidth = bits // 8
            elif chunk_id == b'data':
                data_offset = f.tell()
                data_size = chunk_size
                break
            else:
                f.seek(chunk_size, 1)
        if data_offset is None or rate is None:
            raise ValueError(f'{path}: no data or fmt chunk found')
        f.seek(0, 2)
        eof = f.tell()
        actual_size = eof - data_offset
        if data_size == 0 or data_size > actual_size:
            data_size = actual_size              # header was never finalised
        f.seek(data_offset)
        raw = f.read(data_size)
    dtype = {1: np.int8, 2: np.int16, 4: np.int32}[sampwidth]
    data = np.frombuffer(raw, dtype=dtype)
    if nchannels > 1:
        data = data.reshape(-1, nchannels)[:, 0]
    scale = float(1 << (8 * sampwidth - 1))
    return rate, data.astype(np.float32) / scale

def trim_silence(x, thresh_frac=0.1, win=2048):
    # DC-remove first so a biased ADC doesn't keep |x| above threshold
    # everywhere. Detect signal via sliding RMS so isolated noise-tail
    # samples don't trick argmax into firing at the start of the file.
    x_ac = (x - x.mean()).astype(np.float64)
    if len(x_ac) < win:
        return x, 0
    sq = x_ac * x_ac
    cs = np.concatenate([[0.0], np.cumsum(sq)])
    local_rms = np.sqrt((cs[win:] - cs[:-win]) / win)
    peak = float(local_rms.max())
    if peak <= 0.0:
        return x, 0
    thresh = thresh_frac * peak
    mask = local_rms > thresh
    if not mask.any():
        return x, 0
    # Sliding-RMS index i refers to x[i:i+win]; expand by ~win/2 each
    # side so we don't shave the signal edges.
    pad = win // 2
    i0 = max(0, int(np.argmax(mask)) - pad)
    i1 = min(len(x), len(x) - int(np.argmax(mask[::-1])) + pad)
    return x[i0:i1], i0

def plot_spectrogram(ax, rate, data, title, fmin=20e3, fmax=40e3):
    f, t, Sxx = spectrogram(data, rate, nperseg=2048, noverlap=1536)
    Sxx_db = 10.0 * np.log10(Sxx + 1e-20)
    ax.pcolormesh(t, f, Sxx_db, shading='gouraud',
                  vmin=Sxx_db.max() - 60, vmax=Sxx_db.max())
    ax.set_ylim(fmin, fmax)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Frequency (Hz)')
    ax.set_title(title)

def stats(label, rate, x):
    dc = float(x.mean())
    rms = float(np.sqrt(np.mean(x * x)))
    peak = float(np.max(np.abs(x)))
    # Dominant frequency from FFT
    N = min(len(x), 1 << 20)
    spec = np.abs(np.fft.rfft(x[:N] - dc))
    freqs = np.fft.rfftfreq(N, 1.0 / rate)
    # Only consider 20-40 kHz band
    band = (freqs > 20e3) & (freqs < 40e3)
    dom_idx = int(np.argmax(spec[band]))
    dom_freq = float(freqs[band][dom_idx])
    print(f'{label}: rate={rate} Hz len={len(x)} '
          f'DC={dc:+.4f} RMS={rms:.4f} peak={peak:.4f} '
          f'dom_freq={dom_freq/1e3:.2f} kHz')

def main(paths):
    out_png = os.path.splitext(paths[0])[0] + '_analysis.png'

    if len(paths) == 1:
        rate, x = load(paths[0])
        stats('file', rate, x)
        x, _ = trim_silence(x)
        fig, ax = plt.subplots(1, 1, figsize=(12, 5))
        plot_spectrogram(ax, rate, x, f'{paths[0]} ({rate} Hz, {len(x)} samples)')
        plt.tight_layout()
        plt.savefig(out_png, dpi=120)
        print(f'wrote {out_png}')
        return

    rate_tx, tx = load(paths[0])
    rate_rx, rx = load(paths[1])
    if rate_tx != rate_rx:
        print(f'Sample rates differ: tx={rate_tx} rx={rate_rx}', file=sys.stderr)

    stats(f'TX {paths[0]}', rate_tx, tx)
    stats(f'RX {paths[1]}', rate_rx, rx)

    tx_t, _ = trim_silence(tx)
    rx_t, _ = trim_silence(rx)

    # Cross-correlation to estimate lag. Use a short TX template slid across
    # the full RX file — avoids the O(n²) memory of full×full correlate.
    tmpl_len = min(len(tx_t), 200_000)
    search_len = min(len(rx_t), 4_000_000)
    template = tx_t[:tmpl_len] - tx_t[:tmpl_len].mean()
    search = rx_t[:search_len] - rx_t[:search_len].mean()
    corr = correlate(search, template, mode='valid', method='fft')
    best = int(np.argmax(np.abs(corr)))
    peak = float(corr[best])
    norm = float(np.sqrt(np.sum(template**2) *
                          np.sum(search[best:best + tmpl_len]**2)))
    corr_norm = peak / norm if norm > 0 else 0.0
    lag = best  # template start index in RX

    # Sliding windowed correlation at the best lag — reveals whether the
    # correlation stays strong throughout the message or drops in patches.
    # Window = 4096 samples (~8 ms, several symbol periods).
    win = 4096
    hop = 1024
    if lag + tmpl_len <= len(rx_t):
        rx_aligned = rx_t[lag:lag + tmpl_len] - rx_t[lag:lag + tmpl_len].mean()
        local_corr = []
        local_t = []
        for start in range(0, tmpl_len - win, hop):
            t_win = template[start:start + win]
            r_win = rx_aligned[start:start + win]
            n_t = np.sqrt(np.sum(t_win * t_win))
            n_r = np.sqrt(np.sum(r_win * r_win))
            if n_t > 0 and n_r > 0:
                local_corr.append(float(np.sum(t_win * r_win) / (n_t * n_r)))
            else:
                local_corr.append(0.0)
            local_t.append(start / rate_tx)
        local_corr = np.array(local_corr)
        local_t = np.array(local_t)
        print(f'Local corr (window={win}): '
              f'mean={local_corr.mean():+.3f} '
              f'median={np.median(local_corr):+.3f} '
              f'max={local_corr.max():+.3f} '
              f'min={local_corr.min():+.3f}')
    else:
        local_corr = np.array([])
        local_t = np.array([])
    print(f'Peak cross-correlation: {corr_norm:+.4f} at lag {lag} samples '
          f'({lag / rate_tx * 1e3:+.2f} ms)')
    print(f'TX length: {len(tx_t)} samples ({len(tx_t)/rate_tx:.3f} s)')
    print(f'RX length: {len(rx_t)} samples ({len(rx_t)/rate_rx:.3f} s)')

    fig, axes = plt.subplots(4, 1, figsize=(14, 14))
    plot_spectrogram(axes[0], rate_tx, tx_t, f'TX trimmed: {paths[0]}')
    plot_spectrogram(axes[1], rate_rx, rx_t, f'RX trimmed: {paths[1]}')

    # Cross-correlation curve (template slid across RX search)
    corr_n = np.abs(corr) / (norm + 1e-20)
    lags_ms = np.arange(len(corr)) / rate_tx * 1e3
    mask = lags_ms < 500.0
    axes[2].plot(lags_ms[mask], corr_n[mask], linewidth=0.6)
    axes[2].set_xlabel('Lag (ms)  [template offset into RX]')
    axes[2].set_ylabel('|normalized cross-corr|')
    axes[2].set_title(f'Cross-correlation  (peak {corr_norm:+.4f} at {lag/rate_tx*1e3:+.2f} ms)')
    axes[2].grid(True, alpha=0.3)

    # Local windowed correlation — flat high = good; dropouts/drift show here
    if len(local_corr) > 0:
        axes[3].plot(local_t * 1e3, local_corr, linewidth=0.8)
        axes[3].axhline(0.0, color='k', linewidth=0.3)
        axes[3].set_ylim(-1.05, 1.05)
        axes[3].set_xlabel('Time within message (ms)')
        axes[3].set_ylabel('Windowed normalized correlation')
        axes[3].set_title(f'Local correlation (window {win} samples / {win/rate_tx*1e3:.1f} ms)')
        axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_png, dpi=120)
    print(f'wrote {out_png}')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    main(sys.argv[1:])
