"""Unit tests for the Exp 2 ``Range: %.2fm`` line parser.

CDC lines may carry terminal control sequences, prompt redraws, or
trailing whitespace; the parser must recover the metric value without
false positives on unrelated lines."""
from __future__ import annotations

import pytest

from experiments.exp2_ranging_accuracy import parse_range_line


@pytest.mark.parametrize("line,expected", [
    ("Range: 500.00m",                   500.00),
    ("Range: 487.34m",                   487.34),
    ("Range: 487.34m\r",                 487.34),
    ("...Range: 500.00m...",             500.00),
    ("[123 ns] Range: 502.71m",          502.71),
    ("Range: 0.00m",                       0.00),
    # Tolerate non-conventional whitespace and signed values too.
    ("Range:  -3.14m",                    -3.14),
    ("Range:1234.5m",                   1234.5),
])
def test_well_formed_lines_parse(line: str, expected: float) -> None:
    got = parse_range_line(line)
    assert got is not None
    assert got == pytest.approx(expected, abs=1e-4)


@pytest.mark.parametrize("line", [
    "Range request sent",
    "TXRX > RANGEOUT",
    "Failed ranging request",
    "Received: PROBE 0042",
    "",
    "garbage",
])
def test_unrelated_lines_return_none(line: str) -> None:
    assert parse_range_line(line) is None


def test_non_string_returns_none() -> None:
    assert parse_range_line(None) is None        # type: ignore[arg-type]
    assert parse_range_line(123) is None         # type: ignore[arg-type]
