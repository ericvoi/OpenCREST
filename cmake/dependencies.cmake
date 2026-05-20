include(FetchContent)

# yaml-cpp
FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0)
FetchContent_MakeAvailable(yaml-cpp)

# spdlog
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.14.1)
FetchContent_MakeAvailable(spdlog)

# nlohmann/json — header-only; used by simulator/run_summary for the
# per-run summary JSON consumed by the Python experiment harness.
FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

# libusb (system package — not required for openCREST_core / tests)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(libusb IMPORTED_TARGET libusb-1.0)
    if(libusb_FOUND)
        message(STATUS "libusb-1.0 found via pkg-config")
    else()
        message(STATUS "libusb-1.0 not found — USB transport will not be built")
    endif()
else()
    message(STATUS "PkgConfig not found — USB transport will not be built")
endif()

# Google Test (tests only)
if(OPENCRIEST_BUILD_TESTS)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0)
    # Prevent GoogleTest from overriding compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()
