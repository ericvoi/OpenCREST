include(GoogleTest)

# Test helpers library (signal generators + analysis utilities)
add_library(test_helpers STATIC
    test_helpers/signal_generators.cpp
    test_helpers/analysis.cpp
)
target_include_directories(test_helpers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_helpers PUBLIC openCREST_core GTest::gtest)

# ---------------------------------------------------------------------------
# Macro: add a unit test executable and register it with CTest
# (links openCREST_core + test_helpers — no libusb needed)
# ---------------------------------------------------------------------------
macro(openCREST_unit_test name src)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE openCREST_core test_helpers GTest::gtest_main)
    # POST_BUILD discovery runs the binary at build time; under TSan that
    # fails with the well-known "unexpected memory mapping" kernel ASLR
    # issue. PRE_TEST defers discovery to ctest time, where the test can
    # be run via `setarch -R`.
    if(OPENCRIEST_TSAN)
        gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST)
    else()
        gtest_discover_tests(${name})
    endif()
endmacro()

# ---------------------------------------------------------------------------
# Macro: add an integration test (same deps as unit test — no libusb)
# ---------------------------------------------------------------------------
macro(openCREST_integration_test name src)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE openCREST_core test_helpers GTest::gtest_main)
    if(OPENCRIEST_TSAN)
        gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST)
    else()
        gtest_discover_tests(${name})
    endif()
endmacro()

# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------
openCREST_unit_test(test_ring_buffer        unit/test_ring_buffer.cpp)
openCREST_unit_test(test_sample_conversion  unit/test_sample_conversion.cpp)
openCREST_unit_test(test_farrow_resampler   unit/test_farrow_resampler.cpp)
openCREST_unit_test(test_delay_line         unit/test_delay_line.cpp)
openCREST_unit_test(test_path_loss          unit/test_path_loss.cpp)
openCREST_unit_test(test_noise_generator    unit/test_noise_generator.cpp)
openCREST_unit_test(test_calibration_math   unit/test_calibration_math.cpp)
openCREST_unit_test(test_transducer_response unit/test_transducer_response.cpp)
openCREST_unit_test(test_physical_gain      unit/test_physical_gain.cpp)
openCREST_unit_test(test_noise_psd          unit/test_noise_psd.cpp)
openCREST_unit_test(test_hilbert_filter     unit/test_hilbert_filter.cpp)
openCREST_unit_test(test_channel            unit/test_channel.cpp)
openCREST_unit_test(test_protocol_codec     unit/test_protocol_codec.cpp)
openCREST_unit_test(test_scenario_loader    unit/test_scenario_loader.cpp)
openCREST_unit_test(test_buffer_pacer       unit/test_buffer_pacer.cpp)
openCREST_unit_test(test_pid_fill_tracker_adapter unit/test_pid_fill_tracker_adapter.cpp)
openCREST_unit_test(test_tx_start_estimator unit/test_tx_start_estimator.cpp)
openCREST_unit_test(test_clock_fill_tracker unit/test_clock_fill_tracker.cpp)
openCREST_unit_test(test_arrival_alignment  unit/test_arrival_alignment.cpp)
openCREST_unit_test(test_pair_buffer        unit/test_pair_buffer.cpp)
openCREST_unit_test(test_receiver_mix       unit/test_receiver_mix.cpp)
openCREST_unit_test(test_source_worker      unit/test_source_worker.cpp)
openCREST_unit_test(test_channel_engine_sizing unit/test_channel_engine_sizing.cpp)
openCREST_unit_test(test_geometric_scene    unit/test_geometric_scene.cpp)
openCREST_unit_test(test_channel_geometric  unit/test_channel_geometric.cpp)
openCREST_unit_test(test_source_delay_line  unit/test_source_delay_line.cpp)
openCREST_unit_test(test_per_tap_farrow     unit/test_per_tap_farrow.cpp)
openCREST_unit_test(test_processing_time_stats unit/test_processing_time_stats.cpp)
openCREST_unit_test(test_run_summary        unit/test_run_summary.cpp)

# ---------------------------------------------------------------------------
# Integration tests
# ---------------------------------------------------------------------------
openCREST_integration_test(test_channel_pipeline   integration/test_channel_pipeline.cpp)
openCREST_integration_test(test_channel_geometric_doppler
                            integration/test_channel_geometric_doppler.cpp)
openCREST_integration_test(test_state_transitions  integration/test_state_transitions.cpp)
openCREST_integration_test(test_loopback_mock      integration/test_loopback_mock.cpp)
openCREST_integration_test(test_scenario_files     integration/test_scenario_files.cpp)
openCREST_integration_test(test_noise_levels       integration/test_noise_levels.cpp)
openCREST_integration_test(test_message_event_log  integration/test_message_event_log.cpp)
target_compile_definitions(test_scenario_files
    PRIVATE OPENCREST_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios")
