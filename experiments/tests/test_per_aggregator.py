"""Tests for exp3's PER aggregator and SMS-line parser.

Each cell is one (range, sea_state, seed) tuple; pooling happens across
seeds for each (range, sea_state) point. These tests cover the
pure-function primitives that back the figure and CSV writers.
"""
from __future__ import annotations

import math

import pytest

from experiments.exp3_janus_per import (
    CellResult,
    PooledPoint,
    parse_sms_seq,
    pool_across_seeds,
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
# wilson_ci
# ---------------------------------------------------------------------------

def test_wilson_ci_empty_returns_nan() -> None:
    lo, hi = wilson_ci(0, 0)
    assert math.isnan(lo) and math.isnan(hi)


def test_wilson_ci_brackets_point_estimate() -> None:
    """The point estimate (n_failed/n_total) must lie in the CI."""
    for n_failed, n_total in [(5, 50), (49, 50), (1, 50), (25, 50), (50, 50), (0, 50)]:
        lo, hi = wilson_ci(n_failed, n_total)
        p = n_failed / n_total
        assert lo <= p <= hi, f"{p} not in [{lo}, {hi}] for {n_failed}/{n_total}"


def test_wilson_ci_shrinks_with_n() -> None:
    """Larger sample size -> tighter interval, holding p fixed."""
    lo_small, hi_small = wilson_ci(5,  10)         # p=0.5, n=10
    lo_big,   hi_big   = wilson_ci(50, 100)        # p=0.5, n=100
    assert (hi_big - lo_big) < (hi_small - lo_small)


def test_wilson_ci_clamped_to_unit_interval() -> None:
    """All-success and all-failure cases stay within [0, 1]."""
    lo, hi = wilson_ci(0, 100)
    assert 0.0 <= lo <= hi <= 1.0
    lo, hi = wilson_ci(100, 100)
    assert 0.0 <= lo <= hi <= 1.0


# ---------------------------------------------------------------------------
# CellResult.per
# ---------------------------------------------------------------------------

def test_cell_result_per_all_lost() -> None:
    r = CellResult(range_m=500.0, sea_state=3, seed=0,
                   n_total=10, n_received=0)
    assert r.per == pytest.approx(1.0)


def test_cell_result_per_all_received() -> None:
    r = CellResult(range_m=500.0, sea_state=3, seed=0,
                   n_total=10, n_received=10)
    assert r.per == pytest.approx(0.0)


def test_cell_result_per_half() -> None:
    r = CellResult(range_m=500.0, sea_state=3, seed=0,
                   n_total=20, n_received=10)
    assert r.per == pytest.approx(0.5)


def test_cell_result_per_empty_is_nan() -> None:
    r = CellResult(range_m=500.0, sea_state=3, seed=0,
                   n_total=0, n_received=0)
    assert math.isnan(r.per)


# ---------------------------------------------------------------------------
# pool_across_seeds
# ---------------------------------------------------------------------------

def _cell(range_m: float, ss: int, seed: int, *,
          n_total: int, n_received: int) -> CellResult:
    return CellResult(range_m=range_m, sea_state=ss, seed=seed,
                      n_total=n_total, n_received=n_received)


def test_pool_groups_by_range_and_sea_state() -> None:
    cells = [
        _cell(500.0, 3, 0, n_total=10, n_received=8),
        _cell(500.0, 3, 1, n_total=10, n_received=6),
        _cell(500.0, 3, 2, n_total=10, n_received=7),
        _cell(700.0, 3, 0, n_total=10, n_received=9),
    ]
    pooled = pool_across_seeds(cells)
    by_key = {(p.range_m, p.sea_state): p for p in pooled}
    assert (500.0, 3) in by_key
    assert (700.0, 3) in by_key
    p500 = by_key[(500.0, 3)]
    assert p500.n_total    == 30
    assert p500.n_received == 21
    assert p500.per == pytest.approx(1.0 - 21/30)
    assert isinstance(p500, PooledPoint)


def test_pool_returns_sorted_by_range_then_sea_state() -> None:
    cells = [
        _cell(700.0, 5, 0, n_total=5, n_received=0),
        _cell(300.0, 1, 0, n_total=5, n_received=5),
        _cell(300.0, 3, 0, n_total=5, n_received=4),
        _cell(700.0, 1, 0, n_total=5, n_received=3),
    ]
    pooled = pool_across_seeds(cells)
    keys = [(p.range_m, p.sea_state) for p in pooled]
    assert keys == sorted(keys)


def test_pool_handles_empty_input() -> None:
    assert pool_across_seeds([]) == []


def test_pool_ci_brackets_point_estimate() -> None:
    cells = [_cell(500.0, 3, i, n_total=20, n_received=15) for i in range(5)]
    pooled = pool_across_seeds(cells)
    assert len(pooled) == 1
    p = pooled[0]
    assert p.n_total == 100
    assert p.ci_lo <= p.per <= p.ci_hi


def test_pool_includes_all_seeds_in_pooled_count() -> None:
    cells = [_cell(400.0, 1, s, n_total=15, n_received=10) for s in range(7)]
    pooled = pool_across_seeds(cells)
    assert pooled[0].n_total    == 7 * 15
    assert pooled[0].n_received == 7 * 10
