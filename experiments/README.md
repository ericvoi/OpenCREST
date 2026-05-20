# OpenCREST experiment harness

Python tooling that drives the `openCREST` binary across parameter sweeps,
captures the structured outputs (run-summary JSON, message-event JSONL,
WAVs, CDC console logs), aggregates them into pandas DataFrames, and
renders the paper figures.

This package is the host-side wrapper around `openCREST`. The simulator
itself has no knowledge of this harness — it is just spawned as a
subprocess with a generated scenario YAML. The harness owns:

- Scenario generation (Jinja2 templates → per-cell YAML).
- Subprocess lifecycle (spawn, time-bound, SIGTERM, collect exit code).
- USB CDC console capture (pyserial, opened by the harness in parallel to
  the simulator's bulk-endpoint USB transport).
- Determinism verification across repeat runs.
- Figure rendering.

## Layout

```
experiments/
  README.md                       (this file)
  requirements.txt
  configs/
    common/
      two_modem_base.yaml.j2      base bidirectional scenario template
      golden/
        two_modem_seed42.yaml     pinned render for the template test
  lib/
    scenario_template.py          Jinja2 render + cell-id hash
    runner.py                     Sweep orchestration + subprocess control
    cdc_console.py                /dev/ttyACM* driver + task-level menu helpers
    metrics_loader.py             summary.json / events.jsonl / cdc.log → DataFrames
    wav_io.py                     tolerant WAV reader (handles SIGTERM-truncated files)
    plotting.py                   shared matplotlib helpers (waterfall, ECDF, PER)
    determinism.py                fingerprint diff for repeat runs
  exp1_channel_validation.py      stub; Session F fills in
  exp2_ranging_accuracy.py        stub; Session G fills in
  exp3_janus_per.py               stub; Session H fills in
  tests/
    _stub_binary.py               fake openCREST for hardware-less tests
    test_template.py
    test_runner_smoke.py
    test_cdc_console.py
    test_metrics_loader.py
  results/                        gitignored sweep outputs
```

## Setting up

```bash
python3 -m venv experiments/.venv
experiments/.venv/bin/pip install -r experiments/requirements.txt
```

## Running tests

```bash
experiments/.venv/bin/python -m pytest experiments/tests/ -v
```

These run without any USB hardware attached — they exercise the harness
against `_stub_binary.py`, a fake openCREST that emits artifacts shaped
like the real simulator's Session D outputs.

## Long sweeps (hours-scale): progress + watchdog

For multi-hour sweeps two knobs on `Sweep` matter:

- **`progress=True`** (default) prints one line per cell start and one per
  cell completion to `progress_stream` (`sys.stderr` by default), with a
  running ok/fail count and a rolling ETA based on cells finished so far.
  Set `progress=False` to mute. Sample:

  ```
  [sweep] starting 17 cell(s); upper-bound ~8h30m00s at parallel=1
  [sweep] cell a1b2c3d4e5f6 starting (scheduled 1/17) range_m=200.0 seed=42
  [sweep] cell a1b2c3d4e5f6 ok in 30m02s [1/17 done, 1 ok, 0 fail; eta 8h00m32s]
  ...
  [sweep] complete: 17 ok, 0 fail, total 8h31m04s
  ```

- **`max_cell_runtime_s`** is an optional hard kill timer. It is independent
  of `duration_s` and `stop_condition`: when a cell exceeds it, the runner
  forcibly terminates the process and marks the cell **failed** with
  `error="exceeded max_cell_runtime_s (...s)"`. This is the safety net for
  message-count-driven experiments where `duration_s` is effectively
  infinite — without it, a stuck modem would burn the entire planned budget
  on one stuck cell. Without `max_cell_runtime_s` set, the behaviour is
  the same as before (no watchdog).

### Message-count-driven cells

`stop_condition` is polled once per `poll_interval_s` and receives a
`CellHandle` whose `cell_dir` has the live event JSONL streaming in. The
idiom for "end this cell after 1000 messages":

```python
def stop_when_1000_msgs(handle):
    events = handle.cell_dir / "modem_a_events.jsonl"
    if not events.is_file():
        return False
    with events.open() as fp:
        return sum(1 for _ in fp) >= 1000

Sweep(
    template_path      = "experiments/configs/exp2_ranging.yaml.j2",
    parameters         = {"seed": list(range(10))},
    extra_params       = ...,
    binary             = "/home/ern/Projects/OpenCREST/build/openCREST",
    out_dir            = "experiments/results/exp2",
    duration_s         = float("inf"),
    stop_condition     = stop_when_1000_msgs,
    max_cell_runtime_s = 2 * 3600.0,       # 2 h per cell hard cap
).run()
```

## Running an experiment

```bash
# Real hardware required for the actual sweeps:
experiments/.venv/bin/python -m experiments.exp1_channel_validation --seeds 20
experiments/.venv/bin/python -m experiments.exp2_ranging_accuracy --seeds 10
experiments/.venv/bin/python -m experiments.exp3_janus_per --sea-states 1,3,5
```

Each driver writes per-cell artifacts under `experiments/results/<expN>/<cell_id>/`
and an aggregated `sweep_index.csv` at `experiments/results/<expN>/`. The
figure PDFs land at `experiments/results/<expN>/fig_*.pdf`.

The drivers in Sessions F/G/H are responsible for the experiment-specific
post-processing (xcorr-based IR extraction, ranging two-way arithmetic,
PER binning); the harness only provides the substrate.

## Determinism pre-flight

Before launching an expensive sweep, run the determinism check against a
trivial scenario to confirm the simulator is still reproducible:

```python
from pathlib import Path
from experiments.lib import determinism
from experiments.lib.runner import Sweep

def cell(out):
    return Sweep(
        template_path = "experiments/configs/common/two_modem_base.yaml.j2",
        parameters    = {"seed": [12345]},
        extra_params  = dict(modem_a_serial="OA-2-1", modem_b_serial="OA-2-2",
                             v_radial_m_s=0.0, channel_gain_db=-30.0,
                             wenz_sea_state=2, range_m=150.0),
        binary        = "/home/ern/Projects/OpenCREST/build/openCREST",
        out_dir       = out,
        duration_s    = 30.0,
    ).run()[0].cell_dir

report = determinism.compare_cells(cell(Path("/tmp/det_a")),
                                   cell(Path("/tmp/det_b")))
print(report.to_text())
assert report.ok
```

Per-artifact normalisation rules are documented in
`experiments/lib/determinism.py`. In short: PCM samples are compared
bit-for-bit; CDC-line and event-JSONL fingerprints strip host-side
timestamps; the summary-JSON fingerprint compares a documented
deterministic subset (random_seed, processing-time histogram percentiles,
channel-engine counters).

## Adding a new experiment

1. Author a Jinja2 template under `configs/<expN>/` (or include
   `configs/common/two_modem_base.yaml.j2`).
2. Author `experiments/exp<N>_<name>.py` with a `main(argv)` entry point
   following the existing stubs. The driver should:
   - Build a parameter dict for the sweep.
   - Construct a `Sweep` with that template and parameter cross-product.
   - Optionally attach a `CdcConsole` per modem in `pre_run` to issue
     menu commands and a `post_run` hook to detach.
   - Call `sweep.run()`, then `sweep.collect_results()` to get a
     long-form DataFrame.
   - Post-process and render figures via `lib/plotting`.
3. Add a one-row entry in this README's "Running an experiment" section.

## CDC TTY mapping

`CdcConsole.attach(modem_id, usb_serial, log_path)` walks
`/sys/class/tty/ttyACM*/device/.../serial` and finds the TTY whose USB
descriptor reports the given serial. No YAML changes are needed — the
serial is already in `modems[*].usb_serial`.

If your distro doesn't expose `serial` under `/sys/class/tty/...`, pass
`sys_class_tty=` and `dev_root=` overrides; for hardware-less tests, use
`CdcConsole.attach_backend(modem_id, FakeSerial(), log_path)`.

## Menu vocabulary

The OpenAquatix HMI is digit-driven: each menu shows a numbered list of
children, the user types a number and Enter to descend, or ESC to go
back. `CdcConsole` exposes three task-level helpers rather than a
generic menu-path API:

- `send_chirp_tx()`             → ROOT → DBG (2) → CHIRP_TX (15) → trigger.
- `send_ranging_request(target)` → ROOT → TXRX (4) → RANGEOUT (9) [→ target].
- `send_text_message(text, target)` → ROOT → TXRX (4) → STROUT (3) [→ target] → text.

These are hardcoded against the current firmware menu order (see
`OpenAquatix-Firmware/Application/Src/COMM/comm_*_menu.c`); they will need
updating if the firmware menu is reorganised. One regression per
reorganisation is acceptable, per the Session E plan.
