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
  /home/ubuntu/ode-0.16.1/ode/src
  /home/ubuntu/ode-0.16.1/include
  /home/ubuntu/ode-0.16.1/drawstuff/src
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
  /home/ubuntu/ode-0.16.1/drawstuff/src/.libs/libdrawstuff.a
  /home/ubuntu/ode-0.16.1/ode/src/.libs/libode.a
  X11 GL GLU m pthread rt dl
)


install(TARGETS odesim_node
  DESTINATION lib/${PROJECT_NAME})

# ★ 追加：launch と config のインストール
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()