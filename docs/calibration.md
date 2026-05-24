# Calibration Procedure

OpenCREST builds a physically-traceable signal-level chain from sample-domain
floats at the host through the analog HIL pads, the modem's preamp, and the
acoustic transducer. To do that, it needs a handful of measurements from
each modem at startup. They arrive in a single 64-byte calibration packet on
the control endpoint and are interpreted by `src/dsp/calibration_math.hpp`.

This document describes the wire format, what each field actually measures
(and where in the chain it lives), and the small set of physical
measurements needed to populate them on a new modem.

---

## 1. Calibration packet wire format

Sent on the control IN endpoint in response to `CMD_REQUEST_CALIBRATION`.
Defined in `src/protocol/packets.hpp` (`CalibrationPayload`, exactly 64
bytes, packed little-endian).

| Field | Type | Units | Notes |
|-------|------|-------|-------|
| `type` | `uint8_t` | — | Payload-type discriminator (0x02) |
| `adc_bits` | `uint8_t` | bits | Native ADC resolution; `samples_per_packet()` derives from this |
| `dac_bits` | `uint8_t` | bits | Native DAC resolution |
| `num_input_attenuations` | `uint8_t` | — | Always 2 |
| `noise_floor_psd_counts_per_sqrt_hz` | `float` | counts/√Hz | AFE noise PSD at `center_freq_hz` |
| `loopback_cal_attenuation` | `uint16_t` | — | Index of the `input_attenuation[]` pad active during the loopback gain measurement |
| `loopback_gain` | `float` | linear, sample/sample | End-to-end DAC-sample → ADC-sample gain when looping the DAC back into the receive chain at `loopback_cal_attenuation` |
| `adc_sampling_rate` | `uint32_t` | Hz | Typically 500 000 |
| `dac_sampling_rate` | `uint32_t` | Hz | Typically 500 000; equal to ADC rate |
| `input_attenuation[2]` | `float[2]` | dB | Voltage gain of each selectable RX-pad (negative for resistive dividers) |
| `output_attenuation` | `float` | dB | Voltage gain of the TX-monitor tap (negative — pads the high-voltage drive down to ADC-safe levels) |
| `center_freq_hz` | `float` | Hz | Modem operating center frequency (firmware authoritative) |
| `adc_vref_peak_volts` | `float` | V | Single-ended peak voltage above mid-rail that maps to a sample value of 1.0 |
| `dac_vref_peak_volts` | `float` | V | Same, for the DAC pin |
| `reserved[N]` | `uint8_t[N]` | — | Pads to 64 bytes |

Out-of-range or non-finite floats cause the host to reject the calibration
with an `spdlog::error` — the simulator will not start with bad cal data.

---

## 2. What lives where in the chain

Every field maps to one specific point in the analog signal path. Knowing
which point makes it obvious which physical measurement produces it.

```
                                      ┌──────────────────────────────┐
                                      │            MODEM             │
   Transducer ◄─────── PA ◄────── DAC │ (TX path; not used in HIL)   │
                                      │                              │
   TX monitor ──┐                     │                              │
                ▼                     │                              │
   [output_attenuation] ─► ADC sample │                              │
                                      │                              │
                                      │                              │
   RX inject ──┐                      │                              │
               ▼                      │                              │
   [input_attenuation[k]] ─► Preamp ──┴── ADC ─► samples (RX path)
                              │
                              │  loopback_gain (linear, DAC → ADC)
                              │  measured at input_attenuation[loopback_cal_attenuation]
                              │
                              │  noise_floor_psd_counts_per_sqrt_hz
                              │  measured here (AFE noise at fc, in HIL mode
                              │  with input_attenuation in place)
```

- `output_attenuation` (dB, negative): the TX monitor tap. Lets the host
  recover transducer drive voltage from the captured ADC sample.
- `input_attenuation[0]`, `input_attenuation[1]` (dB, negative): the two
  selectable RX pads. The host pre-multiplies the desired preamp voltage
  by `10^(-atten/20)` before commanding the DAC.
- `loopback_gain` (linear): characterises the *intrinsic preamp gain* by
  feeding the DAC into the RX path through `input_attenuation[loopback_cal_attenuation]`
  with no acoustic source. `preamp_gain_db = 20·log10(loopback_gain) - input_attenuation[cal_idx]`
  (`calibration_math.hpp::preamp_gain_db`).
- `noise_floor_psd_counts_per_sqrt_hz`: PSD of the AFE noise floor in the
  *operating* attenuation configuration. Reported as PSD (not band-integrated
  RMS) so the host can compare it against the simulated Wenz PSD at
  `center_freq_hz` without needing a bandwidth.
- `center_freq_hz`: the modem's operating carrier. Used for
  - path-loss frequency in Thorp absorption,
  - the reference frequency for noise PSD comparisons.
- `adc_vref_peak_volts`, `dac_vref_peak_volts`: anchor sample-domain
  ↔ voltage-domain conversions. A sample value of 1.0 corresponds to this
  many volts above the mid-rail (single-ended, peak convention).

---

## 3. Reference-frame discipline

Every PSD comparison happens at the **preamp input**:

```
preamp_dBV/√Hz = (noise PSD in dB re 1 µPa²/Hz) + RVR
                 ↑ Wenz model output

afe_at_preamp_dBV/√Hz = afe_at_adc_dBFS/√Hz
                       + 20·log10(adc_vref_peak)
                       − preamp_gain_dB
                                   ↑ derived from loopback_gain
```

`compute_boost_db(natural, afe, margin)` in `src/dsp/noise_psd.hpp`
operates in this frame; both inputs must already be referenced to the
preamp.

When you add a new measurement or modify `CalibrationData`, ask: *what
point in the chain does this voltage / PSD live at?* Then keep every value
that gets compared against it in the same frame.

---

## 4. Bring-up procedure for a new modem

The minimum viable cal for a new modem requires four measurements plus
firmware constants. Done once at the bench; the firmware then reports them
on every `CMD_REQUEST_CALIBRATION`.

1. **Transducer model selection** — record which transducer model is wired
   to this modem. The TVR / RVR live in the scenario YAML, not the cal
   packet. The transducer model determines which `transducers.<id>` block
   the scenario must reference.

2. **`output_attenuation`** — apply a known low-frequency sinusoid at the
   PA output (or a known DC level on a bench supply through the protective
   resistor); read the ADC. Compute `output_attenuation = 20·log10(V_adc /
   V_drive)`. Should be a large negative number (typical: −60 dB).

3. **`input_attenuation[0]`, `input_attenuation[1]`** — drive the DAC with
   a known mid-band tone, capture the preamp output, compute attenuation
   for each pad selection. The two pads should differ by ~30 dB.

4. **`loopback_gain` + `loopback_cal_attenuation`** — with no acoustic
   source, command the modem into a self-loopback configuration (DAC →
   selected `input_attenuation` pad → preamp → ADC), inject a known sample
   value, measure the resulting sample value at the ADC. `loopback_gain`
   is the linear sample-to-sample ratio. `loopback_cal_attenuation` records
   which input pad was active. The host derives `preamp_gain_db` from these
   two.

5. **`noise_floor_psd_counts_per_sqrt_hz`** — with the modem in HIL mode
   (operating attenuation in place, no signal), capture a few seconds of
   ADC samples; compute the periodogram and read off the PSD at
   `center_freq_hz` in counts²/Hz; take the square root for counts/√Hz.

6. **`adc_vref_peak_volts`, `dac_vref_peak_volts`** — datasheet values for
   the converter chips, validated against a DC reference at the pins. A
   sample of 1.0 ↔ this many volts above mid-rail.

7. **`center_freq_hz`** — the modem's design center frequency. Authoritative
   from firmware so heterogeneous-fc modem pairs work without scenario
   surgery.

The OpenAquatix firmware computes PSD in `MESS/mess_hil_cal.c` as
`noise_rms_counts / sqrt(measurement_bw_hz)`; check that file when
debugging unexpected cal values.

---

## 5. Verifying calibration on a real modem

1. Boot the simulator with any single-modem loopback scenario. The startup
   log includes a per-channel `physical_gain_dB` line and any
   `boost_db > 0` warning. These two together are a quick sanity check
   that the cal values produce a plausible chain gain.

2. Run with `log_raw_tx: true` and `log_raw_rx: true` in the scenario's
   `logging:` block, then inspect the WAVs with `scripts/analyze_wav.py`:

   - **Multipath delay alignment** — the cross-correlation between TX and
     RX should show distinct arrivals at the configured tap delays.
   - **Path-loss magnitude** — RX peak vs TX peak should match the
     hand-computed Thorp + spreading loss for the configured range and
     `center_freq_hz`.
   - **Doppler** — for a `velocity_radial_m_s ≠ 0` scenario,
     `scripts/verify_doppler_shift.py` confirms the FFT peak shifts by
     `(1 + v/c) × f_tx` (signal-level FFT — packet-success rates are not
     a valid Doppler check).

3. **Noise level cross-check** — the modem-reported in-band RMS during a
   pure-noise interval (before any TX) should match
   `target_psd_dbfs_per_sqrt_hz` × √bandwidth, with the AFE PSD adding
   incoherently below the simulated noise (per the +10 dB default margin).
   If the modem reports noise much louder than expected, the most common
   cause is a chain-gain sign error in `output_attenuation` or one of the
   `input_attenuation[]` values.
