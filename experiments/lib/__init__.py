"""OpenCREST experiment harness.

The ``experiments.lib`` package contains the building blocks that the
per-paper-experiment drivers (``exp1_*``, ``exp2_*``, ``exp3_*``) share:

* :mod:`experiments.lib.scenario_template` -- render Jinja2 templates to YAML
* :mod:`experiments.lib.runner`             -- sweep orchestration + subprocess
* :mod:`experiments.lib.cdc_console`        -- USB CDC TTY driver + task helpers
* :mod:`experiments.lib.metrics_loader`     -- summary/JSONL parsers, dataframes
* :mod:`experiments.lib.wav_io`             -- tolerant WAV reader
* :mod:`experiments.lib.plotting`           -- common matplotlib helpers
* :mod:`experiments.lib.determinism`        -- fingerprint diff for repeat runs

The openCREST binary itself is treated as an opaque subprocess; nothing in this
package modifies its behaviour.
"""
