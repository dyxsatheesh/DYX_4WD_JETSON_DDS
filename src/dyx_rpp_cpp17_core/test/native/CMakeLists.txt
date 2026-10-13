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
