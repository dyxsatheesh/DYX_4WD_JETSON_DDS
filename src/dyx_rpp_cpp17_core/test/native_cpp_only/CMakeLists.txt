cmake_minimum_required(VERSION 3.16)
project(dyx_rpp_cpp17_native_cpp_only LANGUAGES CXX)

# No Python interpreter, Python scripts, pybind, or ROS 2 installation required.
# This is the offline native C++17 smoke lane; the existing reference parity
# lane is intentionally separate and will be retired at final C++ cutover.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()

set(DYX_CPP_ONLY_SMOKE_TESTS
    path_index_native_test
    guidance_mock_smoke
    speed_regulator_native_smoke
    tracking_stability_native_smoke
    tracking_metrics_native_smoke
    stationary_detector_native_smoke
    terminal_stop_native_smoke
    pivot_motion_native_smoke
    forward_yaw_rate_native_smoke
    shadow_geometry_native_smoke)

foreach(test_name IN LISTS DYX_CPP_ONLY_SMOKE_TESTS)
  add_executable(${test_name} ../${test_name}.cpp)
  target_include_directories(${test_name} PRIVATE ../../include)
  if(MSVC)
    target_compile_options(${test_name} PRIVATE /W4)
  else()
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_test(NAME ${test_name} COMMAND "$<TARGET_FILE:${test_name}>")
endforeach()
