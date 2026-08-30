# OpenCREST

OpenCREST (Open Channel Real-time Emulation Streaming Tool) is an underwater acoustic hardware-in-the-loop (HIL) simulator. It captures the analog drive signal from a real modem's transducer over USB, applies a simulated acoustic channel (Doppler, multipath, path loss, ambient noise) on the host PC, and injects the resulting signal back into thereceiving modem in real time.

The minimum 100 ms acoustic propagation delay (150 m at 1500 m/s) absorbs the host's processing latency, so a standard Linux box with a low-latency kernel is enough to make the simulation behave as real-time-equivalent without an FPGA.

## Build

Requires a C++20 compiler, CMake ≥ 3.20, `pkg-config`, and `libusb-1.0` for the host application.

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
| `-DOPENCREST_BUILD_TESTS=OFF` | ON | Skip building tests |
| `-DOPENCREST_ASAN=ON` | OFF | Enable AddressSanitizer |
| `-DOPENCREST_TSAN=ON` | OFF | Enable ThreadSanitizer (see TSan note below) |
| `-DOPENCREST_DEBUG_TALLY=ON` | OFF | Per-tick sample-tally diagnostics in `ChannelEngine` |
| `-DOPENCREST_USE_CLOCK_FILL_TRACKER=OFF` | ON | Use clock-extrapolation buffer-fill tracker + arrival alignment (set OFF to fall back to the PID tracker) |

### TSan on systems with high-entropy ASLR

There have been issues with ThreadSanitizer's shadow-memory map clashing with kernel ASLR. If TSan tests fail with "unexpected memory mapping," run them under `setarch -R`:

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

The simulator prints calibration info, per-channel physical gain, any Wenz-vs-AFE noise boost it applies, and a once-per-second metrics line covering TX/RX packet rates, gaps, and processing-tick latency.

Send `Ctrl-C` for clean shutdown. Logged WAV files (when enabled in the scenario's `logging:` block) land in the configured `output_directory`.

## Hardware Requirements

OpenCREST has been successfully run on the following hardware:
- i5-8500 (6 core CPU from 2018)
- Xeon E5-2699v4 (22 core server CPU from 2016)
- Ryzen 7 7735U (8 core laptop CPU from 2023)

Other CPUs should work as well as long as they are x86-based, released in the past decade, and there is at least 1 core per connected modem.

In addition to the CPU, the host computer should have enough USB ports for each connected modem if using OpenAquatix. WHile this is not a hard requirements, running USB connections through a USB hub can lead to additional latency and jitter. Even if real-time simulation is not an important requirements, USB hubs can lead to addiitonal jitter, especially since no two USB hubs are identical. One point to consider is that on motherboards and cases with many USB ports, many of them are connected to a single USB controller through an internal hub. THis can cause addiitonal latency and, if the motherboard exposes it, try to find USB ports with a dedicated controller.

## Documentation

- [`docs/scenario_reference.md`](docs/scenario_reference.md) — YAML schema reference for scenario files (top-level sections, every field, defaults, and units).
- [`docs/calibration.md`](docs/calibration.md) — calibration packet wire format, what each modem-side field measures, and how `loopback_gain`, AFE noise PSD, and the input-attenuation pads compose into the preamp-referenced gain chain.
- [`docs/metrics_export.md`](docs/metrics_export.md) — per-run JSON / JSONL artifacts produced by the simulator for downstream analysis.
- [`docs/hardware_compatibility.md`](docs/hardware_compatibility.md) — reference modem, interface requirements, and what's needed to add a new transport (USB FS, Ethernet, FPGA bridge, …).
- [`scenarios/`](scenarios/) — example YAML scenarios (single-modem loopback, multi-tap multipath, two-modem, Doppler).
- [`experiments/README.md`](experiments/README.md) — Python harness for parameter sweeps, CDC console capture, and figure rendering.

## Module Layout

```
src/
├── core/         Compile-time constants, types, SPSC ring buffer,
│                  TX-start estimator
├── dsp/          Farrow resampler (bulk + per-tap), Hilbert, multipath
│                  delay-line, source delay-line, path loss, Wenz noise,
│                  noise PSD, physical gain, calibration math, transducer
│                  response
├── channel/      PairBuffer, Channel pipeline, SourceWorker, ReceiverMix,
│                  ChannelEngine, tap sources (geometric scene, replay
│                  trajectories)
├── protocol/     Wire formats + codec for data and control packets
├── transport/    IModemTransport interface; UsbTransport (libusb) and
│                  MockTransport (testing)
├── modem/        Modem state, calibration handshake, registry/discovery
├── io/           Per-modem I/O thread, buffer pacer, clock-extrapolation
│                  and PID fill trackers
├── config/       Scenario YAML loader + validation
├── logging/      WAV writer + per-stream logger
└── simulator/    Top-level coordinator, metrics, processing-time stats,
                   message-event log, run summary
```

`openCREST_core` (the library that all tests link against) has no `libusb` dependency. `openCREST_usb` adds the USB transport; `openCREST_app` ties both together for the executable.

## License

MIT License — see [`LICENSE`](LICENSE).
