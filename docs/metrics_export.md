# Metrics export

When the scenario's `logging` block opts in, the simulator emits structured
per-run artifacts intended for consumption by `experiments/`.
All artifacts share the same `output_directory`.

## YAML controls

```yaml
logging:
  output_directory: results/run_0042
  log_raw_tx:                    false   # existing
  log_raw_rx:                    false   # existing
  log_processed:                 false   # existing
  log_processing_time_histogram: true
  log_message_events:            true
  run_summary_path: ""                   # empty = default
```

Defaults are `false` / empty so existing scenarios produce no new files.

`run_summary_path` defaults to `<output_directory>/<scenario_name>_summary.json`
when empty. Setting it (or either of the histogram / event flags) enables
the summary writer.

## Per-modem message event log

Path: `<output_directory>/<modem_id>_events.jsonl`

One JSON object per line, flushed on every event, so a SIGTERM-ed run
still has up-to-the-last-message data. One event is emitted per TX
message (TX-entry → TX-exit edge pair).

```json
{
  "modem_id":     "modem_a",
  "direction":    "tx",
  "start_ns":     123456789012345,
  "end_ns":       123456789145002,
  "sample_count": 5120,
  "sequence_id":  17
}
```

- `start_ns` / `end_ns`: `steady_clock` nanoseconds; suitable for
  intervals, not wall-clock. The `run_summary.json` `started_at` /
  `ended_at` give wall-clock anchors for the same run.
- `sample_count`: source samples consumed during the TX edge (post-ADC,
  pre-Farrow). The number of receiver-side samples landing in the
  PairBuffer differs from this by the resampler ratio.
- `sequence_id`: monotonic per `(modem, direction)`. Resets per run.

`direction` is currently always `"tx"`.

## Per-run summary

Path: `<output_directory>/<scenario_name>_summary.json`.

```json
{
  "scenario_name":   "geometric_approach",
  "scenario_path":   "experiments/configs/run_0042.yaml",
  "random_seed":     12345,
  "started_at":      "2026-05-18T18:34:21Z",
  "ended_at":        "2026-05-18T18:35:21Z",
  "duration_s":      60.0,
  "modems":          ["modem_a", "modem_b"],
  "processing_time": {
    "count":          60000,
    "mean_us":        312.4,
    "p50_us":         280,
    "p95_us":         540,
    "p99_us":         720,
    "max_us":         1430,
    "underrun_count": 0
  },
  "channel_engine": {
    "rx_ring_underruns": 0,
    "fw_rx_underruns":   0,
    "tx_packets_total":  117612,
    "rx_packets_total":  117612
  },
  "log_files": {
    "events": ["results/run_0042/modem_a_events.jsonl",
               "results/run_0042/modem_b_events.jsonl"],
    "cdc":    [],
    "wav":    []
  }
}
```

`processing_time` records every `SourceWorker::process_available()`
invocation (one per per-source-batch tick). Buckets are log-spaced from
1 µs to 100 ms across 256 slots; percentiles are quoted to bucket-lower-
bound granularity (≤ ~5 % at 1 ms).

`underrun_count` is incremented whenever a tick exceeds the
`PROCESSING_BLOCK_SIZE / sample_rate` deadline.

`log_files.cdc` is populated by the Python experiment harness —
the simulator itself does not read modem CDC.
