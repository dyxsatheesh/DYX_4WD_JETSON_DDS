cmake_minimum_required(VERSION 3.8)
project(dyx_rpp_cpp17_core)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()

# DYX_NATIVE_RPP_REGRESSIONS_V1
if(BUILD_TESTING)
  enable_testing()

  foreach(test_name IN ITEMS
    path_index_native_test
    native_active_leg_integration_test
    native_rpp_executor_test
    native_mission_rpp_pipeline_test
  )
    add_executable(
      ${test_name}
      test/${test_name}.cpp
    )

    target_include_directories(
      ${test_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_compile_options(
      ${test_name} PRIVATE
      -Wall -Wextra -Wpedantic -UNDEBUG
    )

    add_test(
      NAME ${test_name}
      COMMAND ${test_name}
    )
  endforeach()
endif()
