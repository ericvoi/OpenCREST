"""Geometric-twin template + parameter tests."""
from __future__ import annotations

import pytest
import yaml

from experiments.lib import watermark_twins as twins
from experiments.lib.scenario_template import render_template


@pytest.mark.parametrize("dataset", sorted(twins.WATERMARK_TWINS))
def test_twin_renders_to_valid_yaml(dataset):
    params = twins.twin_params(dataset, seed=3)
    params["output_directory"] = "/tmp/out"
    doc = yaml.safe_load(render_template(twins.TEMPLATE, params))

    assert doc["name"] == f"twin_{dataset}_seed3"
    ch = doc["channels"][0]
    g = twins.WATERMARK_TWINS[dataset]
    assert ch["mode"] == "geometric"
    assert ch["range_m"] == pytest.approx(g.range_m)
    assert ch["geometry"]["water_depth_m"] == pytest.approx(g.water_depth_m)
    # Loader constraints: depths inside the water column, R envelope
    # strictly bracketing R_0.
    assert 0.0 < ch["geometry"]["source_depth_m"] < g.water_depth_m
    assert 0.0 < ch["geometry"]["receiver_depth_m"] < g.water_depth_m
    assert ch["geometry"]["r_min_m"] < ch["range_m"] < ch["geometry"]["r_max_m"]


def test_kau_twin_carries_tow_motion():
    params = twins.twin_params("KAU1", run_duration_s=60.0)
    params["output_directory"] = "/tmp/out"
    doc = yaml.safe_load(render_template(twins.TEMPLATE, params))
    v = doc["modems"][0]["velocity_radial_m_s"]
    assert v == pytest.approx(-1.5)   # closing by default
    # Envelope covers the 60 s tow swing (90 m) each side.
    ch = doc["channels"][0]
    assert ch["geometry"]["r_min_m"] <= 1080.0 - 90.0
    assert ch["geometry"]["r_max_m"] >= 1080.0 + 90.0


def test_fixed_datasets_have_no_motion():
    for name in ("NOF1", "NCS1", "BCH1"):
        assert twins.twin_params(name)["velocity_radial_m_s"] == 0.0


def test_receding_option_flips_sign():
    assert twins.twin_params("KAU1", closing=False)[
        "velocity_radial_m_s"] == pytest.approx(1.5)


def test_measured_v0_override_replaces_tow_speed():
    # Signed value straight from the ensemble manifest's mean_v0_m_s.
    params = twins.twin_params("KAU1", run_duration_s=60.0,
                               radial_velocity_override_m_s=-1.14)
    assert params["velocity_radial_m_s"] == pytest.approx(-1.14)
    # Envelope brackets the measured swing (68.4 m over 60 s).
    assert params["r_min_m"] <= 1080.0 - 68.4
    assert params["r_max_m"] >= 1080.0 + 68.4


def test_every_twin_documents_its_assumptions():
    for name, g in twins.WATERMARK_TWINS.items():
        assert "verify" in g.assumptions, name
