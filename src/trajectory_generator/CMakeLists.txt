cmake_minimum_required(VERSION 3.16)
project(trajectory_generator LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(dyx_rpp_cpp17_core REQUIRED)
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
target_include_directories(trajectory_generator_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(trajectory_generator_node rclcpp std_msgs nav_msgs geometry_msgs std_srvs px4_msgs dyx_rpp_cpp17_core)
install(TARGETS trajectory_generator_node DESTINATION lib/${PROJECT_NAME})
ament_package()

# Native C++17 geographic projection and PX4 origin regression.
if(BUILD_TESTING)
  enable_testing()

  add_executable(
    test_geo_origin
    test_geo_origin.cpp
  )

  target_include_directories(
    test_geo_origin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_rpp_cpp17_core/include
  )

  target_compile_options(
    test_geo_origin PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG
  )

  add_test(
    NAME geo_origin_regression
    COMMAND test_geo_origin
  )

  add_executable(test_row_end_skip test_row_end_skip.cpp)
  target_include_directories(test_row_end_skip PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_rpp_cpp17_core/include)
  target_compile_options(test_row_end_skip PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME row_end_skip_regression COMMAND test_row_end_skip)
endif()
