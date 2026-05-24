# Hardware compatibility

OpenCREST is built around a generic transport abstraction
(`IModemTransport`) so that the simulator core does not prescribe a
particular sample rate, packet size, or wire protocol. Any interface that
sustains the required bandwidth and offers a way to time-synchronize with
the simulator can be used.

## Primary target

| Modem | Interface | Sample rate | Notes |
|-------|-----------|-------------|-------|
| [OpenAquatix](https://github.com/OpenAquatix) | USB HS vendor-class bulk endpoints | 500 kSPS | Reference hardware. Used by the in-tree `UsbTransport` and validated end-to-end with two-way ranging and JANUS-style messaging. |

The OpenAquatix interface contract is small and modem-agnostic: two
512-byte bulk endpoints carry sample streams (one direction each), and
two 64-byte bulk endpoints carry control / status / calibration packets.
See [`docs/calibration.md`](calibration.md) and
[`docs/scenario_reference.md`](scenario_reference.md) for the wire
format and how scenarios refer to specific modems by USB serial.

## Other interfaces

Any link that meets the following requirements can host a new
`IModemTransport` implementation without changes to the channel
pipeline, DSP, or simulator core:

- **Bandwidth.** At least `2 × sample_rate × sample_width × N_modems` of
  full-duplex throughput for the sample streams, plus a low-rate side
  channel for control and status. At 500 kSPS / 16-bit / 2 modems this
  is ~4 MB/s, well under USB HS, USB FS (in principle), Gigabit
  Ethernet, or a 10 GbE link.
- **Time synchronization.** The simulator needs a way to anchor sample
  positions to a wall-clock instant on each side of the link — either an
  in-band timestamp on sample packets, an out-of-band sync mechanism
  (PTP, shared GPIO, etc.), or an explicit start-of-transmission packet
  whose host arrival time can be used as the reference. Without this,
  propagation delay and inter-modem arrival alignment cannot be
  reconstructed.
- **Two-way framing.** Sample packets must carry enough framing to
  detect drops and re-establish alignment, since the simulator's
  fill-tracker assumes monotonic sample-stream progress.

Candidate transports include:

- **USB FS.** Lower bandwidth (~1 MB/s practical bulk ceiling) and
  larger per-packet timing jitter; viable for lower-rate scenarios.
- **Ethernet / UDP.** Requires application-level framing and either
  in-band timestamps or PTP for synchronization. Removes the per-host
  modem-count limit imposed by USB controllers.
- **Custom FPGA bridges.** A streaming bridge that exposes the same
  `IModemTransport` interface can front-end any acoustic modem with the
  necessary analog tap and injection points.

A new transport is added by implementing `IModemTransport` (see
`src/transport/modem_transport.hpp`) and adapting modem discovery /
calibration handshake to the new link. The DSP, channel, and scenario
layers are unaffected.
