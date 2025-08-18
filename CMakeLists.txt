cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)
project(message_tf_frame_transformer)

## Compile as C++17
add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(route_planning_msgs REQUIRED)
find_package(route_planning_msgs_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_perception_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_route_planning_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs_utils REQUIRED)

add_executable(${PROJECT_NAME} src/message_tf_frame_transformer.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  perception_msgs
  perception_msgs_utils
  rclcpp
  route_planning_msgs
  route_planning_msgs_utils
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_perception_msgs
  tf2_ros
  tf2_route_planning_msgs
  tf2_sensor_msgs
  tf2_trajectory_planning_msgs
  trajectory_planning_msgs
  trajectory_planning_msgs_utils
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
