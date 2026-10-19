include(FetchContent)
FetchContent_Declare(
  raylib
  URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
find_package(raylib REQUIRED)

add_library(asv_viewer raylib_viewer_sink.cpp)
target_include_directories(asv_viewer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(asv_viewer PUBLIC asv_core raylib)

add_executable(sim_viewer sim_viewer.cpp)
target_link_libraries(sim_viewer PRIVATE asv_viewer)