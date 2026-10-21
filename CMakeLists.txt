cmake_minimum_required(VERSION 3.20)
project(asv_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release for single-config generators (Makefiles, Ninja)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Use extraction-time timestamps for downloaded archives (CMake >= 3.24)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

# Project-wide warnings, linked explicitly so third-party code isn't affected
add_library(asv_warnings INTERFACE)
target_compile_options(asv_warnings INTERFACE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /utf-8>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)
target_compile_definitions(asv_warnings INTERFACE
  $<$<CXX_COMPILER_ID:MSVC>:NOMINMAX _CRT_SECURE_NO_WARNINGS>
)

# Core library (headless)
add_library(asv_core
  src/vessel.cpp
  src/heading_controller.cpp
  src/los_guidance.cpp
  src/csv_frame_sink.cpp
  src/radar_sensor.cpp
  src/ais_sensor.cpp
  src/cpa.cpp
)
target_include_directories(asv_core PUBLIC include)
target_link_libraries(asv_core PUBLIC asv_warnings)

# GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_subdirectory(tests)
add_subdirectory(apps)

option(ASV_BUILD_VIEWER "Build the raylib viewer" ON)
if(ASV_BUILD_VIEWER)
  add_subdirectory(viewer)
endif()

target_sources(asv_core PRIVATE src/target_ship.cpp)