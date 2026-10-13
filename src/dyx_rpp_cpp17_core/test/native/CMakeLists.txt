cmake_minimum_required(VERSION 3.16)
project(dyx_rpp_cpp17_native LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(rpp_probe ../rpp_probe.cpp)
target_include_directories(rpp_probe PRIVATE ../../include)
enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME python_cpp_parity
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../python_parity.py" "$<TARGET_FILE:rpp_probe>")

# Phase 2B: full index/semantic span/progress parity with Python RPP.
add_executable(geometry_index_probe ../geometry_index_probe.cpp)
target_include_directories(geometry_index_probe PRIVATE ../../include)
add_test(NAME python_cpp_path_index_parity
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../geometry_index_parity.py" "$<TARGET_FILE:geometry_index_probe>")

add_executable(path_index_native_test ../path_index_native_test.cpp)
target_include_directories(path_index_native_test PRIVATE ../../include)
add_test(NAME path_index_native_smoke COMMAND "$<TARGET_FILE:path_index_native_test>")

# Phase 2C1: Python/C++17 full steering-target calculation parity.
add_executable(guidance_probe ../guidance_probe.cpp)
target_include_directories(guidance_probe PRIVATE ../../include)
add_test(NAME python_cpp_precision_guidance_parity
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../guidance_parity.py" "$<TARGET_FILE:guidance_probe>")

add_executable(guidance_mock_smoke ../guidance_mock_smoke.cpp)
target_include_directories(guidance_mock_smoke PRIVATE ../../include)
add_test(NAME guidance_mock_smoke COMMAND "$<TARGET_FILE:guidance_mock_smoke>")

# Phase 2C2: longitudinal state machine and Python reference replay.
add_executable(speed_regulator_probe ../speed_regulator_probe.cpp)
target_include_directories(speed_regulator_probe PRIVATE ../../include)
add_test(NAME python_cpp_speed_regulator_parity
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../speed_regulator_parity.py" "$<TARGET_FILE:speed_regulator_probe>")

add_executable(speed_regulator_native_smoke ../speed_regulator_native_smoke.cpp)
target_include_directories(speed_regulator_native_smoke PRIVATE ../../include)
add_test(NAME speed_regulator_native_smoke COMMAND "$<TARGET_FILE:speed_regulator_native_smoke>")

# Phase 2C3: stateful tracking recovery and recapture hysteresis parity.
add_executable(tracking_stability_probe ../tracking_stability_probe.cpp)
target_include_directories(tracking_stability_probe PRIVATE ../../include)
add_test(NAME python_cpp_tracking_stability_parity
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../tracking_stability_parity.py" "$<TARGET_FILE:tracking_stability_probe>")

add_executable(tracking_stability_native_smoke ../tracking_stability_native_smoke.cpp)
target_include_directories(tracking_stability_native_smoke PRIVATE ../../include)
add_test(NAME tracking_stability_native_smoke COMMAND "$<TARGET_FILE:tracking_stability_native_smoke>")
