cmake_minimum_required(VERSION 3.8)
project(wall_follower_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(wall_follower_node
  src/wall_follower.cpp
)

ament_target_dependencies(wall_follower_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS wall_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

