"""Render Jinja2 scenario templates to concrete YAML.

Templates live under ``experiments/configs/`` with the ``.yaml.j2``
extension. Each template takes a parameter dict and produces a scenario
YAML that the openCREST binary can consume directly. The renderer uses
``StrictUndefined`` so missing parameters fail loudly.

The rendered output is byte-stable for a given parameter dict.
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
    ``{% include %}`` each other."""
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
    """Stable short identifier derived from the parameter dict (SHA256 over
    sorted-key JSON, truncated to 12 hex chars)."""
    canonical = json.dumps(params, sort_keys=True, separators=(",", ":"),
                           default=str).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()[:12]


def validate_yaml(text: str) -> dict:
    """Parse ``text`` as YAML and return the resulting dict. Raises
    ``yaml.YAMLError`` on malformed input or ``ValueError`` if the result
    is not a mapping.
    """
    obj = yaml.safe_load(text)
    if not isinstance(obj, dict):
        raise ValueError("rendered template did not produce a YAML mapping")
    return obj
