cmake_minimum_required(VERSION 3.8)
project(my_ode_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ODE / drawstuff のパス（あなたの環境に合わせて調整）
include_directories(
  include
  include/my_ode_sim
  /home/yuhei/ode-0.16.2/ode/src
  /home/yuhei/ode-0.16.2/include
  /home/yuhei/ode-0.16.2/drawstuff/src
)

add_executable(odesim_node
  src/main.cpp
  src/localization.cpp
  src/robot.cpp
  src/lidar.cpp
  src/world.cpp
)

ament_target_dependencies(odesim_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ODE / drawstuff ライブラリ（v05 で使っていたものと同じ）
target_link_libraries(odesim_node
  /home/yuhei/ode-0.16.2/drawstuff/src/.libs/libdrawstuff.a
  /home/yuhei/ode-0.16.2/ode/src/.libs/libode.a
  X11 GL GLU m pthread rt dl
)


install(TARGETS odesim_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()