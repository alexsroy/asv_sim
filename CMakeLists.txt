cmake_minimum_required(VERSION 3.20)
project(asv_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Core library (headless). Empty for now, filled in during step 2.
add_library(asv_core src/vessel.cpp src/heading_controller.cpp)
target_include_directories(asv_core PUBLIC include)
target_compile_options(asv_core PRIVATE -Wall -Wextra -Wpedantic)

# GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_subdirectory(tests)