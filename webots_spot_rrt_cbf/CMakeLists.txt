cmake_minimum_required(VERSION 3.16)
project(webots_spot_rrt_cbf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(webots_spot_msgs REQUIRED)

# ROS-free core (vendored from rrt_star_cbf/cpp_version) + adapters.
add_library(safe_cbf_rrt_core STATIC
  src/safe_cbf_rrt/grid.cpp
  src/safe_cbf_rrt/planner.cpp
  src/field_adapter.cpp
)
target_include_directories(safe_cbf_rrt_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(safe_cbf_rrt_core PRIVATE -Wall -Wextra)

add_library(rrt_cbf_planner SHARED src/rrt_cbf_planner.cpp)
target_compile_options(rrt_cbf_planner PRIVATE -Wall -Wextra)
ament_target_dependencies(rrt_cbf_planner
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util pluginlib
  nav_msgs geometry_msgs visualization_msgs std_msgs std_srvs tf2_ros webots_spot_msgs
)
target_link_libraries(rrt_cbf_planner safe_cbf_rrt_core)
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS rrt_cbf_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/planner_benchmark.py scripts/path_capture.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/bench_common.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safe_cbf_rrt test/test_safe_cbf_rrt.cpp)
  target_link_libraries(test_safe_cbf_rrt safe_cbf_rrt_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(rrt_cbf_planner)
ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d pluginlib)
ament_package()
