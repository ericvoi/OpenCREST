# OpenCREST

OpenCREST (Open Channel Real-time Emulation Streaming Tool) is an underwater acoustic hardware-in-the-loop (HIL) simulator. It
captures the analog drive signal from a real modem's transducer over USB,
applies a simulated acoustic channel (Doppler, multipath, path loss, ambient
noise) on the host PC, and injects the resulting signal back into the
receiving modem in real time.

The minimum 100 ms acoustic propagation delay (150 m at 1500 m/s) absorbs the
host's processing latency, so a standard Linux box with a low-latency kernel
is enough to make the simulation behave as real-time-equivalent — no FPGA
required.

## Status

- Single-modem loopback runs end-to-end on real hardware.
- Two-modem mock-transport tests pass; multi-modem hardware bring-up is
  in progress.
- Physically-traceable signal levels (TVR / RVR / preamp-referenced AFE PSD)
  are wired through the entire pipeline.
- 290+ unit/integration tests link against `openCREST_core` and run without
  USB hardware.

## Build

Requires a C++20 compiler, CMake ≥ 3.20, `pkg-config`, and `libusb-1.0` for
the host application (the core library and tests build without `libusb`).

```bash
# Install system dependencies (Debian/Ubuntu)
sudo ./scripts/install-deps.sh

# Configure + build (RelWithDebInfo)
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j$(nproc)
```

Optional CMake flags:

| Flag | Default | Purpose |
|------|---------|---------|
| `-DOPENCRIEST_BUILD_TESTS=OFF` | ON | Skip building tests |
| `-DOPENCRIEST_ASAN=ON` | OFF | Enable AddressSanitizer |
| `-DOPENCRIEST_TSAN=ON` | OFF | Enable ThreadSanitizer (see TSan note below) |
| `-DOPENCREST_DEBUG_TALLY=ON` | OFF | Per-tick sample-tally diagnostics in `ChannelEngine` |

### TSan on systems with high-entropy ASLR

ThreadSanitizer's shadow-memory map clashes with kernel ASLR on some Linux
hosts. If TSan tests fail with "unexpected memory mapping," run them under
`setarch -R`:

```bash
setarch $(uname -m) -R ctest --test-dir build --output-on-failure
```

## Test

```bash
ctest --test-dir build --output-on-failure

# Or run a single binary:
./build/tests/test_channel
./build/tests/test_channel --gtest_filter="Channel.Acceleration*"
```

## Run

```bash
./build/openCREST scenarios/loopback_simple.yaml
```

The simulator prints calibration info, per-channel physical gain, any
Wenz-vs-AFE noise boost it applies, and a once-per-second metrics line
covering TX/RX packet rates, gaps, and processing-tick latency.

Send `Ctrl-C` for clean shutdown. Logged WAV files (when enabled in the
scenario's `logging:` block) land in the configured `output_directory`.

## Documentation

- [`docs/scenario_reference.md`](docs/scenario_reference.md) — YAML schema
  reference for scenario files (top-level sections, every field, defaults,
  and units).
- [`docs/calibration.md`](docs/calibration.md) — calibration packet wire
  format, what each modem-side field measures, and how `loopback_gain`,
  AFE noise PSD, and the input-attenuation pads compose into the
  preamp-referenced gain chain.
- [`spec.md`](spec.md) — system-level specification.
- [`architecture.md`](architecture.md) — implementation architecture
  (modules, threading, hot-path constraints).
- [`firmware_spec.md`](firmware_spec.md) — modem firmware contract.
- [`scenarios/`](scenarios/) — example YAML scenarios (single-modem
  loopback, multi-tap multipath, two-modem, Doppler).

## Module Layout

```
src/
├── core/         Compile-time constants, types, SPSC ring buffer
├── dsp/          Farrow resampler, Hilbert, multipath delay-line, path loss,
│                  Wenz noise, calibration math, transducer response
├── channel/      PairBuffer, Channel pipeline, SourceWorker, ReceiverMix,
│                  ChannelEngine
├── protocol/     Wire formats + codec for data and control packets
├── transport/    IModemTransport interface; UsbTransport (libusb) and
│                  MockTransport (testing)
├── modem/        Modem state, calibration handshake, registry/discovery
├── io/           Per-modem I/O thread, buffer pacer
├── config/       Scenario YAML loader + validation
├── logging/      WAV writer + per-stream logger
└── simulator/    Top-level coordinator, metrics
```

`openCREST_core` (the library that all tests link against) has no `libusb`
dependency. `openCREST_usb` adds the USB transport; `openCREST_app` ties
both together for the executable.

## License

See `LICENSE`
