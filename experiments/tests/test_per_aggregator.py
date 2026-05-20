"""Tests for exp3's PER-vs-range aggregator and SMS-line parser.

Session H plan: bin TX records by their per-packet range-at-TX into the
chosen range bins (default 100 m wide, 200→1000 m = 8 bins). Per bin,
PER = 1 − (received / total); empty bins report NaN, not zero.
"""
from __future__ import annotations

import math

import pytest

from experiments.exp3_janus_per import (
    DEFAULT_BIN_EDGES_M,
    PerBin,
    TxRecord,
    aggregate_per,
    parse_sms_seq,
    wilson_ci,
)


# ---------------------------------------------------------------------------
# parse_sms_seq
# ---------------------------------------------------------------------------

def test_parse_sms_seq_extracts_zero_padded_number() -> None:
    assert parse_sms_seq("SMS: PROBE 042") == 42


def test_parse_sms_seq_handles_leading_whitespace() -> None:
    assert parse_sms_seq("    SMS:   PROBE 7") == 7


def test_parse_sms_seq_returns_none_for_other_lines() -> None:
    for line in [
        "Sender i.d.: 73",
        "SMS: hi",                                # not a PROBE payload
        "Received a new message at 237s",
        "Range: 487.20m",
        "",
    ]:
        assert parse_sms_seq(line) is None, f"unexpected match: {line!r}"


def test_parse_sms_seq_handles_three_digit_seq() -> None:
    assert parse_sms_seq("SMS: PROBE 199") == 199


def test_parse_sms_seq_ignores_non_strings() -> None:
    assert parse_sms_seq(None) is None       # type: ignore[arg-type]
    assert parse_sms_seq(42)   is None       # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# aggregate_per
# ---------------------------------------------------------------------------

def _rec(rng: float, received: bool, *, rid: int = 0) -> TxRecord:
    return TxRecord(
        request_id        = rid,
        payload           = f"PROBE {rid:03d}",
        tx_event_start_ns = 0,
        range_at_tx_m     = rng,
        received          = received,
    )


def test_default_bin_edges_cover_200_to_1000_in_100m_steps() -> None:
    """The driver's default partitioning is 200..1000 in 100 m steps."""
    assert DEFAULT_BIN_EDGES_M == [200.0, 300.0, 400.0, 500.0,
                                   600.0, 700.0, 800.0, 900.0, 1000.0]


def test_per_one_when_all_lost() -> None:
    recs = [_rec(450.0, False, rid=i) for i in range(20)]
    bins = aggregate_per(recs, DEFAULT_BIN_EDGES_M)
    bin400 = next(b for b in bins if b.lo_m == 400.0)
    assert bin400.n_total    == 20
    assert bin400.n_received == 0
    assert bin400.per        == pytest.approx(1.0)


def test_per_zero_when_all_received() -> None:
    recs = [_rec(750.0, True, rid=i) for i in range(10)]
    bins = aggregate_per(recs, DEFAULT_BIN_EDGES_M)
    bin700 = next(b for b in bins if b.lo_m == 700.0)
    assert bin700.n_total    == 10
    assert bin700.n_received == 10
    assert bin700.per        == pytest.approx(0.0)


def test_per_half_when_half_received() -> None:
    recs = (
        [_rec(550.0, True,  rid=i) for i in range(50)]
        + [_rec(550.0, False, rid=i + 50) for i in range(50)]
    )
    bins = aggregate_per(recs, DEFAULT_BIN_EDGES_M)
    bin500 = next(b for b in bins if b.lo_m == 500.0)
    assert bin500.n_total == 100
    assert bin500.per     == pytest.approx(0.5)


def test_empty_bin_reports_nan() -> None:
    """Range-at-TX values land in only some bins; the rest must report NaN."""
    recs = [_rec(950.0, True, rid=0)]
    bins = aggregate_per(recs, DEFAULT_BIN_EDGES_M)
    bin200 = next(b for b in bins if b.lo_m == 200.0)
    assert bin200.n_total == 0
    assert math.isnan(bin200.per)


def test_records_outside_edges_are_dropped() -> None:
    """A record at 1500 m (above the highest bin edge) shouldn't crash and
    shouldn't get added to any bin."""
    recs = [_rec(1500.0, True, rid=0),
            _rec(50.0,   True, rid=1),       # below lowest edge
            _rec(500.0,  True, rid=2)]
    bins = aggregate_per(recs, DEFAULT_BIN_EDGES_M)
    totals = sum(b.n_total for b in bins)
    assert totals == 1
    bin500 = next(b for b in bins if b.lo_m == 500.0)
    assert bin500.n_total == 1


def test_lower_edge_inclusive_upper_exclusive() -> None:
    """A record at exactly a bin's lower edge belongs to that bin; one at
    the upper edge belongs to the next bin (standard half-open convention)."""
    recs = [_rec(400.0, True, rid=0),       # belongs to [400, 500)
            _rec(500.0, True, rid=1)]       # belongs to [500, 600)
    bins = aggregate_per(recs, DEFAULT_BIN_EDGES_M)
    by_lo = {b.lo_m: b for b in bins}
    assert by_lo[400.0].n_total == 1
    assert by_lo[500.0].n_total == 1


# ---------------------------------------------------------------------------
# wilson_ci
# ---------------------------------------------------------------------------

def test_wilson_ci_empty_returns_nan() -> None:
    lo, hi = wilson_ci(0, 0)
    assert math.isnan(lo) and math.isnan(hi)


def test_wilson_ci_brackets_point_estimate() -> None:
    """Whatever the CI is, the point estimate (n_failed/n_total) must lie in it."""
    for n_failed, n_total in [(5, 50), (49, 50), (1, 50), (25, 50), (50, 50), (0, 50)]:
        lo, hi = wilson_ci(n_failed, n_total)
        p = n_failed / n_total
        assert lo <= p <= hi, f"{p} not in [{lo}, {hi}] for {n_failed}/{n_total}"


def test_wilson_ci_shrinks_with_n() -> None:
    """Larger sample size → tighter interval, holding p fixed."""
    lo_small, hi_small = wilson_ci(5,  10)         # p=0.5, n=10
    lo_big,   hi_big   = wilson_ci(50, 100)        # p=0.5, n=100
    width_small = hi_small - lo_small
    width_big   = hi_big   - lo_big
    assert width_big < width_small


def test_wilson_ci_clamped_to_unit_interval() -> None:
    """All-success and all-failure cases stay inside [0, 1]."""
    lo, hi = wilson_ci(0, 100)
    assert 0.0 <= lo <= hi <= 1.0
    lo, hi = wilson_ci(100, 100)
    assert 0.0 <= lo <= hi <= 1.0


def test_returned_bins_cover_all_edges_in_order() -> None:
    bins = aggregate_per([], DEFAULT_BIN_EDGES_M)
    # 8 bins for 200..1000 step 100
    assert len(bins) == len(DEFAULT_BIN_EDGES_M) - 1
    assert [b.lo_m for b in bins] == DEFAULT_BIN_EDGES_M[:-1]
    assert [b.hi_m for b in bins] == DEFAULT_BIN_EDGES_M[1:]
    for b in bins:
        assert isinstance(b, PerBin)
        assert b.n_total == 0
        assert math.isnan(b.per)
