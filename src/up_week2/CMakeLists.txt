cmake_minimum_required(VERSION 3.8)
project(up_week2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(includes)

add_executable(target_mover
  code/Mover/main.cpp
  code/Mover/TargetMover.cpp)
ament_target_dependencies(target_mover rclcpp)

install(TARGETS target_mover
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY worlds config DESTINATION share/${PROJECT_NAME})

ament_package()
