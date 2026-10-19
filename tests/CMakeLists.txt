add_executable(asv_tests
  smoke_test.cpp
  vessel_test.cpp
  heading_controller_test.cpp
  los_guidance_test.cpp
  csv_frame_sink_test.cpp
  view_transform_test.cpp
)

target_link_libraries(asv_tests PRIVATE asv_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(asv_tests)