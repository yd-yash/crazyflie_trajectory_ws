cmake_minimum_required(VERSION 3.8)
project(crazyflie_gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

# --- Crazyflie Motor Plugin ---
add_library(cf_motor_plugin SHARED
  src/cf_motor_plugin.cpp
)

target_include_directories(cf_motor_plugin PUBLIC
  include
  ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(cf_motor_plugin
  ${GAZEBO_LIBRARIES}
)

ament_target_dependencies(cf_motor_plugin
  rclcpp
  gazebo_ros
  std_msgs
)

install(TARGETS cf_motor_plugin
  LIBRARY DESTINATION lib
)

# --- Install directories ---
install(DIRECTORY
  launch
  worlds
  models
  urdf
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

