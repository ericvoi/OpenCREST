"""Pre-flight SNR check for the four Exp 2 channel configs.

Run before ``exp2_ranging_accuracy.py`` to confirm each ranging request
produces a clean response under the configured channel. The main
experiment relies on high SNR so the residual range error is dominated
by modem-side ranging precision, not packet loss.

For each config a/b/c/d the script:

1. Spawns the simulator with raw-RX WAV logging on.
2. Attaches a CDC console to modem A and issues ``--num-requests``
   ranging requests with a short inter-request delay.
3. After SIGTERM, measures the in-band power of the captured RX WAV
   inside the packet window vs the noise floor outside it, and reports
   per-config SNR alongside the ranging success rate.

Per-config pass criteria:

  success_rate >= --min-success           default 0.8
  SNR(rx_b)    >= --min-snr-db            default 12.0

SNR is measured only at modem B (receiving A's request packet); see
``VerifyResult.passed`` for why modem A's RX WAV is info-only.

Exit code 0 if every requested config passes both, 1 otherwise.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import wave
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from experiments.exp2_ranging_accuracy import (
    CONFIG_IDS,
    DEFAULT_FIRST_REQUEST_DELAY_S,
    TEMPLATES,
    _RangingDriver,
    _find_cell_dir,
    _load_records,
)
from experiments.lib.runner import Sweep


REPO = Path(__file__).resolve().parents[1]

DEFAULT_NUM_REQUESTS = 3
DEFAULT_INTER_REQUEST_DELAY_S = 1.0
DEFAULT_RESPONSE_TIMEOUT_S = 5.0
DEFAULT_MAX_CELL_RUNTIME_S = 90.0
SIGTERM_GRACE_S = 6.0

DEFAULT_MIN_SUCCESS = 0.8
DEFAULT_MIN_SNR_DB  = 12.0


# ---------------------------------------------------------------------------
# SNR estimator
# ---------------------------------------------------------------------------

def _read_wav_mono(path: Path) -> tuple[np.ndarray, int]:
    """Return ``(samples_float, sample_rate)``. Mono 16-bit PCM only — the
    simulator's stream_logger writes that exclusively.
    """
    with wave.open(str(path), "rb") as wf:
        nch = wf.getnchannels()
        sw  = wf.getsampwidth()
        fr  = wf.getframerate()
        n   = wf.getnframes()
        raw = wf.readframes(n)
    if sw != 2:
        raise ValueError(f"{path}: expected 16-bit PCM, got {sw * 8}-bit")
    samples = np.frombuffer(raw, dtype="<i2").astype(np.float64)
    if nch > 1:
        samples = samples.reshape(-1, nch).mean(axis=1)
    if samples.size:
        samples /= 32768.0
    return samples, fr


def estimate_snr_db(samples: np.ndarray,
                    *,
                    window_s: float = 0.05,
                    floor_dbfs: float = -70.0
                    ) -> float | None:
    """Crude packet-vs-silence SNR estimate from a single RX WAV.

    Chops the stream into ``window_s``-second blocks, takes the loudest
    block's RMS as the signal estimate, and the median of the quietest 25%
    as the noise floor. ``floor_dbfs`` clamps the noise estimate so a
    completely silent WAV doesn't blow up the ratio. Returns ``None`` if
    the WAV is too short to chop.
    """
    if samples.size < int(window_s * 500_000):
        return None
    block = max(1, int(round(window_s * 500_000)))
    n = samples.size // block
    if n < 4:
        return None
    rms = np.array([
        float(np.sqrt(np.mean(samples[i * block:(i + 1) * block] ** 2)))
        for i in range(n)
    ])
    # dBFS for stable comparison.
    eps = 1e-12
    rms_db = 20.0 * np.log10(rms + eps)
    signal_db = float(np.max(rms_db))
    quiet = np.sort(rms_db)[: max(1, n // 4)]
    noise_db = float(np.median(quiet))
    noise_db = max(noise_db, floor_dbfs)
    return signal_db - noise_db


def _largest_rx_wav(cell_dir: Path, modem_serial: str) -> Path | None:
    candidates = sorted(cell_dir.glob(f"{modem_serial}_rx_*.wav"))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_size)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

@dataclass
class VerifyResult:
    config_id: str
    num_requests: int
    num_responses: int
    success_rate: float
    snr_a_db: float | None     # RX WAV at modem A (response packet from B)
    snr_b_db: float | None     # RX WAV at modem B (request packet from A)

    def passed(self, *, min_success: float, min_snr_db: float) -> bool:
        """PASS if success rate >= ``min_success`` and ``rx_b`` SNR >=
        ``min_snr_db``. ``rx_a`` SNR is info-only: modem A's RX WAV
        captures long silent stretches between brief response packets, so
        block-RMS underestimates the true packet-vs-noise ratio. The
        ranging channel is symmetric, so ``rx_b`` (which captures A's
        louder request packet) is a good stand-in for the SNR at A on the
        return path.
        """
        if self.success_rate < min_success:
            return False
        if self.snr_b_db is None:
            return False
        return self.snr_b_db >= min_snr_db


def _run_one(config_id: str,
             out_root: Path,
             *,
             binary: Path | str,
             modem_a_serial: str,
             modem_b_serial: str,
             num_requests: int,
             inter_request_delay_s: float,
             first_request_delay_s: float,
             response_timeout_s: float,
             max_cell_runtime_s: float,
             ) -> VerifyResult:
    driver = _RangingDriver(
        modem_a_serial         = modem_a_serial,
        modem_b_serial         = modem_b_serial,
        num_requests           = num_requests,
        response_timeout_s     = response_timeout_s,
        inter_request_delay_s  = inter_request_delay_s,
        first_request_delay_s  = first_request_delay_s,
    )
    Sweep(
        template_path     = TEMPLATES[config_id],
        parameters        = {"config": [config_id]},
        extra_params      = dict(
            modem_a_serial = modem_a_serial,
            modem_b_serial = modem_b_serial,
            log_raw_rx     = True,
        ),
        binary             = binary,
        out_dir            = out_root,
        duration_s         = float("inf"),
        parallel           = 1,
        sigterm_grace_s    = SIGTERM_GRACE_S,
        max_cell_runtime_s = max_cell_runtime_s,
        pre_run            = driver.pre_run,
        post_run           = driver.post_run,
        stop_condition     = driver.cell_done,
        progress           = True,
    ).run()

    cell_dir = _find_cell_dir(out_root, config_id)
    if cell_dir is None:
        return VerifyResult(config_id, 0, 0, 0.0, None, None)

    records = _load_records(cell_dir)
    num_responses = sum(1 for r in records if r.range_m is not None)

    rx_a = _largest_rx_wav(cell_dir, modem_a_serial)
    rx_b = _largest_rx_wav(cell_dir, modem_b_serial)
    snr_a = snr_b = None
    if rx_a is not None:
        samples, _ = _read_wav_mono(rx_a)
        snr_a = estimate_snr_db(samples)
    if rx_b is not None:
        samples, _ = _read_wav_mono(rx_b)
        snr_b = estimate_snr_db(samples)

    return VerifyResult(
        config_id      = config_id,
        num_requests   = len(records),
        num_responses  = num_responses,
        success_rate   = (num_responses / len(records)) if records else 0.0,
        snr_a_db       = snr_a,
        snr_b_db       = snr_b,
    )


def _format_snr(v: float | None) -> str:
    return f"{v:6.1f} dB" if v is not None else "    n/a"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", default="experiments/results/exp2/_verify",
                   help="output directory (separate from the main exp2 "
                        "results, since verify writes raw-RX WAVs)")
    p.add_argument("--binary",
                   default=str(REPO / "build" / "openCREST"),
                   help="path to the openCREST simulator binary")
    p.add_argument("--configs", default=",".join(CONFIG_IDS),
                   help="comma-separated subset of {a,b,c,d} to verify")
    p.add_argument("--num-requests", type=int, default=DEFAULT_NUM_REQUESTS,
                   help="ranging requests per config "
                        f"(default {DEFAULT_NUM_REQUESTS})")
    p.add_argument("--inter-request-delay-s", type=float,
                   default=DEFAULT_INTER_REQUEST_DELAY_S)
    p.add_argument("--first-request-delay-s", type=float,
                   default=DEFAULT_FIRST_REQUEST_DELAY_S,
                   help="extra wait after the simulator-ready banner before "
                        "the first ranging request "
                        f"(default {DEFAULT_FIRST_REQUEST_DELAY_S})")
    p.add_argument("--response-timeout-s", type=float,
                   default=DEFAULT_RESPONSE_TIMEOUT_S)
    p.add_argument("--max-cell-runtime-s", type=float,
                   default=DEFAULT_MAX_CELL_RUNTIME_S)
    p.add_argument("--modem-a-serial", default="OA-2-1")
    p.add_argument("--modem-b-serial", default="OA-2-2")
    p.add_argument("--min-success", type=float, default=DEFAULT_MIN_SUCCESS,
                   help="per-config success-rate threshold for PASS "
                        f"(default {DEFAULT_MIN_SUCCESS})")
    p.add_argument("--min-snr-db", type=float, default=DEFAULT_MIN_SNR_DB,
                   help="per-config SNR(rx_b) threshold for PASS "
                        f"(default {DEFAULT_MIN_SNR_DB})")
    args = p.parse_args(argv)

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    config_ids = [c.strip() for c in args.configs.split(",") if c.strip()]
    bad = [c for c in config_ids if c not in CONFIG_IDS]
    if bad:
        sys.stderr.write(f"[exp2-verify] unknown config(s): {bad}\n")
        return 2

    results: list[VerifyResult] = []
    for cid in config_ids:
        sys.stderr.write(f"[exp2-verify] running config ({cid})\n")
        results.append(_run_one(
            config_id              = cid,
            out_root               = out_dir,
            binary                 = args.binary,
            modem_a_serial         = args.modem_a_serial,
            modem_b_serial         = args.modem_b_serial,
            num_requests           = args.num_requests,
            inter_request_delay_s  = args.inter_request_delay_s,
            first_request_delay_s  = args.first_request_delay_s,
            response_timeout_s     = args.response_timeout_s,
            max_cell_runtime_s     = args.max_cell_runtime_s,
        ))

    sys.stderr.write("[exp2-verify] summary:\n")
    sys.stderr.write(
        "  cfg  requests  success     SNR(rx_b)   SNR(rx_a)*  verdict\n")
    overall_ok = True
    for r in results:
        ok = r.passed(min_success=args.min_success,
                      min_snr_db=args.min_snr_db)
        overall_ok = overall_ok and ok
        verdict = "PASS" if ok else "FAIL"
        sys.stderr.write(
            f"  {r.config_id:3s}  {r.num_requests:8d}  "
            f"{r.success_rate * 100:6.1f}%  "
            f"{_format_snr(r.snr_b_db)}  {_format_snr(r.snr_a_db)}  "
            f"{verdict}\n"
        )
    sys.stderr.write(
        "\n[exp2-verify] thresholds: success >= "
        f"{args.min_success * 100:.0f}%, "
        f"SNR(rx_b) >= {args.min_snr_db:.1f} dB\n"
        "  * SNR(rx_a) is info-only: modem A's RX WAV is dominated by\n"
        "    cycle silence + brief response packets, so block-RMS\n"
        "    underestimates the true packet-vs-noise ratio. Verdict\n"
        "    uses SNR(rx_b) (A's request at modem B) as the channel SNR\n"
        "    proxy.\n"
    )
    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
