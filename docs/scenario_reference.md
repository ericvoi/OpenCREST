# Scenario YAML Reference

Scenario files describe everything OpenCREST needs to simulate one HIL run: the
modems involved, the channel between them, the noise environment, and what
gets logged. The loader (`src/config/scenario_loader.cpp`) parses, validates,
and freezes the config at startup — no field changes mid-run.

A scenario file is a YAML mapping with these top-level sections:

| Section | Required | Purpose |
|---------|----------|---------|
| `name` | yes | Human-readable scenario id |
| `description` | no | Free text |
| `random_seed` | no | uint64; controls per-modem `actual_clock_offset_ppm` draw |
| `environment` | no | Sound speed, water type, spreading model, buffer sizing |
| `transducers` | yes¹ | TVR / RVR per transducer model |
| `modems` | yes | One entry per modem in the scenario |
| `channels` | yes | One entry per directed source→receiver path |
| `noise` | no | Wenz model parameters and tonal sources |
| `logging` | no | Raw-stream WAV/binary logging |

¹ Required because every `modem` must reference a `transducer_id`. If you
don't define any transducers the loader rejects every modem.

---

## `environment`

```yaml
environment:
  sound_speed_m_s: 1500.0          # default 1500
  saltwater: true                  # default true; affects Thorp absorption
                                    # and also the noise model unless
                                    # overridden in `noise.saltwater`
  spreading_model: spherical       # spherical | cylindrical | hybrid
                                    # → spreading_factor 2 / 1 / 1.5
  spreading_factor: 2.0            # explicit override (preferred over model)
  max_range_m: 0.0                 # 0 → derive from largest channel.range_m
                                    # used to size every PairBuffer
  max_message_duration_s: 10.0     # longest single message; sizes per-pair
                                    # in-flight buffer (10 s × 500 kSPS =
                                    # 5 M float = 20 MB per pair, by default)
```

`environment.center_freq_khz` is **rejected** — center frequency is
per-modem and comes from the firmware-reported calibration packet.

---

## `transducers`

```yaml
transducers:
  T-001:
    tvr_db: 152.0   # Transmit Voltage Response: dB re 1 µPa/V @ 1 m
    rvr_db: -180.0  # Receive Voltage Response: dB re 1 V/µPa
  T-002:
    tvr_db: 150.0
    rvr_db: -178.0
```

MVP ships flat (frequency-independent) values. The `TransducerResponse`
abstract interface is in place so a future per-frequency lookup table or FIR
drops in without touching call sites.

The loader validates that every `modems[].transducer_id` refers to an entry
defined here.

---

## `modems`

```yaml
modems:
  - id: modem_a                    # required; must be unique
    usb_serial: "OA-2-1"           # required; matched by ModemRegistry
    transducer_id: "T-001"         # required; references transducers section
    clock_offset_ppm: 2.5          # σ for truncated-normal draw of
                                    # actual_clock_offset_ppm; ≥ 0; default 0
    velocity_radial_m_s: 0.0       # initial radial velocity, + = closing
    acceleration_radial_m_s2: 0.0  # constant radial acceleration; spec
                                    # §6.2.3; the Farrow ratio is updated
                                    # per processing block from
                                    # v(t) = v0 + a·t over the message
                                    # duration
```

`clock_offset_ppm` is a **tolerance**, not a signed offset: at scenario load
the loader draws each modem's `actual_clock_offset_ppm` from a truncated
normal with σ = `clock_offset_ppm`. The same `random_seed` reproduces the
same draws. The relative offset on a directed channel is `src.actual −
rx.actual`, so loopback channels always have zero clock offset.

Maximum number of modems is `MAX_MODEMS` (`src/core/constants.hpp`,
currently 6).

---

## `channels`

One entry per **directed** source→receiver path. A bidirectional pair needs
two entries.

```yaml
channels:
  - from: modem_a                  # required; must reference a defined modem
    to:   modem_b                  # required; same
    range_m: 500.0                 # > 0; default 150 m
                                    # used for path-loss spreading; with
                                    # propagation_delay_s < 0 also drives
                                    # the base propagation delay
    gain_db: 0.0                   # additive trim folded into every tap;
                                    # use to compensate empirical chain
                                    # gain until firmware-reported voltages
                                    # match the YAML transducer model
    rx_atten_idx: 1                # 0 or 1; selects which entry of the
                                    # receiver's input_attenuation[] is
                                    # un-done in the gain chain. Default 1
                                    # matches OpenAquatix HIL behaviour
                                    # (firmware forces ATTENUATION_63DB).
    direct_los: true               # if false, no zero-delay tap may be
                                    # specified and at least one tap must
                                    # exist (the earliest is the first
                                    # arrival)
    propagation_delay_s: -1.0      # negative → derive from range_m /
                                    # sound_speed_m_s. Set ≥ 0 to override
                                    # for refracting / non-geometric rays.
    multipath_taps:                # optional; if omitted and direct_los
                                    # is true, a single zero-delay unity
                                    # tap is auto-added
      - delay_s: 0.000             # EXCESS over the direct path (seconds)
        gain_db: 0.0               # dB relative to the main tap
        phase_deg: 0.0             # complex tap phase; 0 keeps the channel
                                    # on the cheap real-only scatter path
                                    # (any non-zero phase enables the
                                    # 31-tap Hilbert pair, adding a 15-
                                    # sample group delay to every tap)
      - delay_s: 0.004
        gain_db: -6.0
        phase_deg: 180.0           # surface bounce inverts phase
      - delay_s: 0.011
        gain_db: -10.0
        phase_deg: 0.0
```

**Tap delay convention.** `delay_s` is excess delay over the direct path
(zero for the direct ray). The base propagation delay (`range_m /
sound_speed_m_s`, or `propagation_delay_s` if set) is applied automatically.

**Tap gain convention.** `gain_db` is **dB relative to the main (zero-delay)
tap**. Path loss (Thorp absorption + spreading) is computed at the source
modem's `center_freq_hz` and folded into the channel gain via the physical
chain — you do not need to subtract path loss from the tap gains.

**Inter-tap timing.** For a tap at `delay_s = 0.004 s` on a 500 kSPS modem,
the receiver sees that arrival 2000 samples after the direct path. The
delay quantises to integer samples at `round(delay_s × Fs)`.

Constraints (loader-enforced):

- `MAX_TAPS_PER_CHANNEL` (32) taps per channel
- `0 ≤ delay_s ≤ MAX_MULTIPATH_DELAY_S` (200 ms)
- `direct_los: false` requires at least one explicit non-zero-delay tap
- `range_m > 0`

### `mode: geometric` — method-of-images scene with R(t) evolution

A channel may opt in to a **method-of-images** propagation model instead of
the flat tap list above. In `mode: geometric`, taps are recomputed at every
processing block from the instantaneous horizontal range
`R(t) = initial_range_m + v·t + 0.5·a·t²` (v / a sourced from the source
modem). Doppler from radial motion emerges **naturally** as the direct-path
`delta_samples` shrinks (closing) or grows (opening); the bulk Farrow
resampler ratio drops its `(1 + v/c)` factor in geometric mode (clock-offset
still applies). Sub-sample Doppler refinement is a Session-C concern.

```yaml
channels:
  - from: modem_a
    to:   modem_b
    range_m: 1000.0                 # PairBuffer sizing reference
    mode: geometric                 # opt in to the scene model
    initial_range_m: 1000.0         # R_0; falls back to range_m if omitted
    geometry:
      water_depth_m: 120.0          # required
      source_depth_m: 50.0          # required, 0 < z_s < water_depth_m
      receiver_depth_m: 100.0       # required, 0 < z_r < water_depth_m
      gamma_surface: -0.9           # pressure-release: −1; default −1
      gamma_bottom:   0.7           # mid-mud reference 0.5; default 0.5
      spreading_exponent_k: 1.5     # k in eq.(2); default 1.5
      enable_direct: true           # all default true except the two below
      enable_surface: true
      enable_bottom: true
      enable_surface_bottom: false
      enable_bottom_surface: false
      r_min_m: 500.0                # default R_0 / 2; lower envelope bound
      r_max_m: 1100.0               # default R_0 * 2; upper envelope bound
```

`multipath_taps:` may **not** be specified alongside `mode: geometric`; the
loader rejects the collision. Up to five enabled paths are produced per
block, sorted by length ascending so the direct path (when enabled) is
first. Tap amplitudes follow paper eq.(2): `(∏ Γ) / ℓ^k · 10^(−α(f_c)·ℓ /
20000)` (Thorp absorption at the source modem's `center_freq_hz`); the AFE
voltage / TVR / RVR chain multiplies on top. `r_min_m` and `r_max_m`
bracket the range R(t) is allowed to sweep — `PairBuffer` capacity is
sized from these bounds and R(t) is clamped to them at runtime.

Constraints (loader-enforced):

- `water_depth_m > 0`; `source_depth_m`, `receiver_depth_m` ∈ (0, `water_depth_m`)
- `spreading_exponent_k > 0`
- `r_min_m < R_0 < r_max_m` (strict)
- `multipath_taps` empty (use static mode for hand-tuned taps)

---

## `noise`

```yaml
noise:
  wenz_sea_state: 3                # 0..6; drives the wind-noise PSD curve
  min_margin_above_afe_db: 10.0    # required PSD margin (at the receiver's
                                    # center frequency, preamp-referenced)
                                    # over the modem-reported AFE noise.
                                    # If natural Wenz < AFE + margin, every
                                    # channel feeding that receiver is
                                    # boosted by the deficit (and the
                                    # ambient noise too, preserving SNR).
                                    # Boost > 0 logs a startup warning.
  disable: false                   # short-circuit ambient noise to zero
                                    # (still applies tonal sources)
  saltwater: true                  # noise-model water type; defaults to
                                    # environment.saltwater
  tonal_sources:                   # added on top of shaped ambient
    - frequency_hz: 50.0           # power-line interference example
      amplitude_linear: 0.005      # peak amplitude in DAC-fraction
      bandwidth_hz: 0.5            # 0 → pure tone; > 0 → narrowband
                                    # noise of given bandwidth
```

`noise.level_above_noise_floor_db` is **rejected** by the loader (the
schema breaking change landed in Phase C). Replace with
`min_margin_above_afe_db` and/or `disable`.

The boost decision happens per receiver, not per channel. If two channels
land into the same receiver, they share the same boost.

---

## `logging`

```yaml
logging:
  log_raw_tx: true                 # capture per-modem TX (modem→host)
  log_raw_rx: true                 # capture per-modem RX (host→modem)
  log_processed: false             # post-channel pre-noise pre-DAC
  output_directory: "logs"
  file_format: wav                 # wav | raw
```

WAV files use the modem-reported sample rate (typically 500 kSPS). File
names include the modem id, stream direction, and a timestamp.

---

## Validation behaviour

- Every parse error throws `openCREST::ScenarioLoadError` with a message
  pointing at the offending field.
- Schema regressions (legacy keys, missing requireds) are explicit errors,
  not silent defaults — the loader is intentionally conservative because a
  bad scenario can produce signals that look plausible but are physically
  wrong.

---

## Recipe: minimal loopback

```yaml
name: my_loopback

transducers:
  T-001:
    tvr_db: 150.0
    rvr_db: -180.0

modems:
  - id: m
    usb_serial: "OA-2-1"
    transducer_id: T-001

channels:
  - from: m
    to: m
    range_m: 150.0       # 100 ms one-way at 1500 m/s
    gain_db: -30.0       # trim until firmware cal matches T-001 spec
```

That's the smallest scenario the loader accepts. Path loss, Doppler (zero),
default 10 dB margin above AFE noise, and a direct-only multipath tap are
all auto-derived.
