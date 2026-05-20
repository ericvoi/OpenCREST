"""Render Jinja2 scenario templates to concrete YAML.

The experiment templates live under ``experiments/configs/`` and follow the
``.yaml.j2`` extension. Each template takes a parameter dict and produces a
scenario YAML that the openCREST binary can consume directly. The renderer
uses ``StrictUndefined`` so missing parameters fail loudly instead of silently
emitting an empty cell.

The rendered output is byte-stable for a given parameter dict, which lets us
checksum/golden-test it without flakiness.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import jinja2
import yaml


def _env_for_path(template_path: Path) -> jinja2.Environment:
    """Build a Jinja2 env rooted at the template's parent so siblings can
    ``{% include %}`` each other (used by ``configs/common/`` snippets)."""
    return jinja2.Environment(
        loader=jinja2.FileSystemLoader([
            str(template_path.parent),
            str(template_path.parent.parent),
            str(template_path.parent.parent.parent),
        ]),
        undefined=jinja2.StrictUndefined,
        keep_trailing_newline=True,
        autoescape=False,
        trim_blocks=False,
        lstrip_blocks=False,
    )


def render_template(template_path: str | Path,
                    params: Mapping[str, Any]) -> str:
    """Render ``template_path`` with ``params``. Returns the YAML text."""
    path = Path(template_path).resolve()
    env = _env_for_path(path)
    template = env.get_template(path.name)
    return template.render(**dict(params))


def render_to_file(template_path: str | Path,
                   params: Mapping[str, Any],
                   out_path: str | Path) -> Path:
    """Render and write to ``out_path``. Creates parent dirs. Returns the
    written path."""
    text = render_template(template_path, params)
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    return out


def cell_id(params: Mapping[str, Any]) -> str:
    """Stable short identifier derived from the parameter dict.

    The hash is computed over the JSON form with sorted keys so equivalent
    dicts collide. Truncated to 12 hex chars to keep paths readable.
    """
    canonical = json.dumps(params, sort_keys=True, separators=(",", ":"),
                           default=str).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()[:12]


def validate_yaml(text: str) -> dict:
    """Parse ``text`` as YAML and return the resulting dict.

    Lets the caller assert that a rendered template is at least well-formed
    YAML before handing it to the simulator. Raises ``yaml.YAMLError`` on
    malformed input.
    """
    obj = yaml.safe_load(text)
    if not isinstance(obj, dict):
        raise ValueError("rendered template did not produce a YAML mapping")
    return obj
