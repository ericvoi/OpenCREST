"""Golden test for the base scenario template.

The template's render must be byte-identical for a fixed parameter dict.
This is the spec's only documented invariant for ``scenario_template.py``
and the foundation for the rest of the harness (the cell_id hash, the
determinism check, and the experiment templates all rely on it).
"""
from __future__ import annotations

from pathlib import Path

import pytest

from experiments.lib import scenario_template as st


ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / "configs" / "common" / "two_modem_base.yaml.j2"
GOLDEN   = ROOT / "configs" / "common" / "golden" / "two_modem_seed42.yaml"


GOLDEN_PARAMS = dict(
    seed=42,
    modem_a_serial="OA-2-1",
    modem_b_serial="OA-2-2",
    range_m=500.0,
    v_radial_m_s=-2.0,
    channel_gain_db=-30.0,
    wenz_sea_state=3,
    output_directory="experiments/results/golden/cell_seed42",
)


def test_render_matches_golden() -> None:
    rendered = st.render_template(TEMPLATE, GOLDEN_PARAMS)
    expected = GOLDEN.read_text(encoding="utf-8")
    assert rendered == expected, (
        "Rendered template diverged from the golden file. If the divergence "
        "is intentional, regenerate the golden via "
        "`python -m experiments.tools.regen_golden`."
    )


def test_render_is_valid_yaml_with_expected_top_level_keys() -> None:
    rendered = st.render_template(TEMPLATE, GOLDEN_PARAMS)
    parsed = st.validate_yaml(rendered)
    for key in ("name", "environment", "modems", "channels",
                "noise", "logging", "random_seed", "transducers"):
        assert key in parsed, f"missing top-level key: {key}"
    assert parsed["random_seed"] == 42
    assert len(parsed["modems"]) == 2
    assert len(parsed["channels"]) == 2


def test_render_is_deterministic_across_repeats() -> None:
    a = st.render_template(TEMPLATE, GOLDEN_PARAMS)
    b = st.render_template(TEMPLATE, GOLDEN_PARAMS)
    assert a == b


def test_cell_id_is_stable_and_distinct() -> None:
    a = dict(GOLDEN_PARAMS); a["seed"] = 1
    b = dict(GOLDEN_PARAMS); b["seed"] = 2
    assert st.cell_id(a) != st.cell_id(b)
    assert st.cell_id(a) == st.cell_id(dict(a))
    # Permuting dict order must not change the id (sorted-keys canonical form).
    permuted = {k: a[k] for k in reversed(list(a.keys()))}
    assert st.cell_id(a) == st.cell_id(permuted)


def test_strict_undefined_raises_on_missing_param() -> None:
    incomplete = dict(GOLDEN_PARAMS)
    incomplete.pop("seed")
    with pytest.raises(Exception):
        st.render_template(TEMPLATE, incomplete)
