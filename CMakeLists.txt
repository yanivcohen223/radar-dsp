cmake_minimum_required(VERSION 3.20)
project(radar_dsp VERSION 1.0.0 LANGUAGES CXX)

# ─── Language standard ───────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── Build options ───────────────────────────────────────────────────────────
option(ENABLE_PROFILING "Enable per-stage performance counters" ON)
option(ENABLE_ASAN "Enable AddressSanitizer (debug builds)" OFF)

# ─── Compiler flags ──────────────────────────────────────────────────────────
add_compile_options(
    -Wall -Wextra -Werror
    -O3
    -pthread
)

# march=native only on native builds; skip under QEMU / CI cross-compile
if(NOT DEFINED ENV{CI})
    add_compile_options(-march=native)
endif()

if(ENABLE_PROFILING)
    add_compile_definitions(ENABLE_PROFILING)
endif()

if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# ─── Find dependencies ───────────────────────────────────────────────────────
find_package(Threads REQUIRED)

# FFTW3 (single-precision)
find_library(FFTW3F_LIB NAMES fftw3f REQUIRED)
find_path(FFTW3_INCLUDE_DIR NAMES fftw3.h REQUIRED)

# Curses
find_package(Curses REQUIRED)

# GTest
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    find_library(GTEST_LIB      NAMES gtest      PATHS /usr/lib /usr/local/lib)
    find_library(GTEST_MAIN_LIB NAMES gtest_main PATHS /usr/lib /usr/local/lib)
    find_path(GTEST_INCLUDE_DIR NAMES gtest/gtest.h
              PATHS /usr/include /usr/local/include)
    if(GTEST_LIB AND GTEST_MAIN_LIB AND GTEST_INCLUDE_DIR)
        add_library(GTest::GTest UNKNOWN IMPORTED)
        set_target_properties(GTest::GTest PROPERTIES
            IMPORTED_LOCATION "${GTEST_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIR}")
        add_library(GTest::Main UNKNOWN IMPORTED)
        set_target_properties(GTest::Main PROPERTIES
            IMPORTED_LOCATION "${GTEST_MAIN_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIR}")
        set(GTest_FOUND TRUE)
    endif()
endif()

# ─── Source files (flat src/ layout) ─────────────────────────────────────────
set(DSP_SOURCES
    src/adc_simulator.cpp
    src/waveform_generator.cpp
    src/pulse_compressor.cpp
    src/range_doppler.cpp
    src/cfar_detector.cpp
    src/rt_thread.cpp
    src/radar_pipeline.cpp
    src/track_manager.cpp
    src/perf_counter.cpp
    src/terminal_dashboard.cpp
)

# ─── Main library (shared between binary and tests) ──────────────────────────
add_library(radar_dsp_lib STATIC ${DSP_SOURCES})

target_include_directories(radar_dsp_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${FFTW3_INCLUDE_DIR}
    ${CURSES_INCLUDE_DIR}
)

target_link_libraries(radar_dsp_lib PUBLIC
    Threads::Threads
    ${FFTW3F_LIB}
    ${CURSES_LIBRARIES}
)

# ─── Main executable ─────────────────────────────────────────────────────────
add_executable(radar_dsp src/main.cpp)
target_link_libraries(radar_dsp PRIVATE radar_dsp_lib)

# ─── Tests ───────────────────────────────────────────────────────────────────
if(GTest_FOUND)
    enable_testing()

    set(TEST_SOURCES
        tests/test_pulse_compression.cpp
        tests/test_range_doppler.cpp
        tests/test_cfar.cpp
        tests/test_tracker.cpp
    )

    add_executable(run_tests ${TEST_SOURCES})
    target_include_directories(run_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${FFTW3_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIR}
    )
    target_link_libraries(run_tests PRIVATE
        radar_dsp_lib
        GTest::GTest
        GTest::Main
    )

    add_test(NAME radar_tests COMMAND run_tests)
else()
    message(WARNING "GTest not found — tests will not be built")
endif()
