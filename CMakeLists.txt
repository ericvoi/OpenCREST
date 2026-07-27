cmake_minimum_required(VERSION 3.20)
project(OpenCREST VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(OPENCREST_BUILD_TESTS  "Build unit and integration tests"          ON)
option(OPENCREST_ASAN         "Enable AddressSanitizer"                   OFF)
option(OPENCREST_TSAN         "Enable ThreadSanitizer"                    OFF)
option(OPENCREST_DEBUG_TALLY   "Enable per-tick sample tally diagnostics in ChannelEngine" OFF)
option(OPENCREST_USE_CLOCK_FILL_TRACKER
       "Use clock-extrapolation buffer-fill tracker + arrival-alignment instead of PID"
       ON)

if(OPENCREST_ASAN AND OPENCREST_TSAN)
    message(FATAL_ERROR "OPENCREST_ASAN and OPENCREST_TSAN cannot both be enabled simultaneously")
endif()

if(OPENCREST_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

if(OPENCREST_TSAN)
    add_compile_options(-fsanitize=thread)
    add_link_options(-fsanitize=thread)
endif()

include(cmake/dependencies.cmake)

# ---------------------------------------------------------------------------
# openCREST_core — no USB dependency; all tests link against this
#
# Contains: DSP, channel engine, protocol codec, config loader, mock transport,
#           modem state machine (IModemTransport), I/O thread, buffer pacer,
#           logging (WAV writer + stream logger).
# ---------------------------------------------------------------------------
add_library(openCREST_core STATIC
    # DSP
    src/dsp/wenz_model.cpp
    src/dsp/path_loss.cpp
    src/dsp/farrow_resampler.cpp
    src/dsp/delay_line.cpp
    src/dsp/noise_generator.cpp
    src/dsp/transducer_response.cpp
    src/dsp/hilbert_filter.cpp
    src/dsp/source_delay_line.cpp
    src/dsp/per_tap_farrow.cpp
    # Core
    src/core/tap_trajectory.cpp
    # Channel
    src/channel/pair_buffer.cpp
    src/channel/channel.cpp
    src/channel/geometric_scene.cpp
    src/channel/geometric_tap_source.cpp
    src/channel/replay_tap_source.cpp
    src/channel/receiver_mix.cpp
    src/channel/source_worker.cpp
    src/channel/channel_engine.cpp
    # Protocol
    src/protocol/protocol_codec.cpp
    # Transport
    src/transport/mock_transport.cpp
    # Config
    src/config/scenario_loader.cpp
    # Modem (uses IModemTransport — no libusb)
    src/modem/modem.cpp
    # I/O
    src/io/buffer_pacer.cpp
    src/io/pid_fill_tracker.cpp
    src/io/clock_fill_tracker.cpp
    src/core/tx_start_estimator.cpp
    src/io/modem_io.cpp
    # Logging
    src/logging/wav_writer.cpp
    src/logging/stream_logger.cpp
    # Metrics (struct only, no USB)
    src/simulator/metrics.cpp
    src/simulator/processing_time_stats.cpp
    src/simulator/message_event_log.cpp
    src/simulator/run_summary.cpp
)
target_include_directories(openCREST_core PUBLIC src/)
target_link_libraries(openCREST_core PUBLIC spdlog::spdlog yaml-cpp nlohmann_json::nlohmann_json)
target_compile_options(openCREST_core PRIVATE -Wall -Wextra -Wpedantic)
if(OPENCREST_DEBUG_TALLY)
    target_compile_definitions(openCREST_core PUBLIC OPENCREST_DEBUG_TALLY)
endif()
if(OPENCREST_USE_CLOCK_FILL_TRACKER)
    target_compile_definitions(openCREST_core
        PUBLIC OPENCREST_USE_CLOCK_FILL_TRACKER)
endif()

# ---------------------------------------------------------------------------
# openCREST_usb — links libusb; only built when libusb is available
# ---------------------------------------------------------------------------
if(libusb_FOUND)
    add_library(openCREST_usb STATIC
        src/transport/usb_transport.cpp
        src/transport/usb_device_discovery.cpp
    )
    target_link_libraries(openCREST_usb PUBLIC openCREST_core PkgConfig::libusb)
    target_compile_options(openCREST_usb PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# openCREST_app — full simulator (requires libusb for ModemRegistry + Simulator)
# ---------------------------------------------------------------------------
if(libusb_FOUND)
    add_library(openCREST_app STATIC
        src/modem/modem_registry.cpp   # default factory creates UsbTransport
        src/simulator/simulator.cpp
    )
    target_link_libraries(openCREST_app PUBLIC openCREST_core openCREST_usb)
    target_compile_options(openCREST_app PRIVATE -Wall -Wextra -Wpedantic)

    add_executable(openCREST src/main.cpp)
    target_link_libraries(openCREST PRIVATE openCREST_app)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(OPENCREST_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
