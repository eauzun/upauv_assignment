cmake_minimum_required(VERSION 3.8)
project(up_week2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

include_directories(includes)

add_executable(target_mover
  code/Mover/main.cpp
  code/Mover/TargetMover.cpp)
ament_target_dependencies(target_mover rclcpp geometry_msgs)
target_link_libraries(target_mover
  ignition-transport11::core
  ignition-msgs8::core)

add_executable(red_target_detector
  code/Detector/main.cpp
  code/Detector/RedTargetDetector.cpp)
ament_target_dependencies(red_target_detector
  rclcpp std_msgs geometry_msgs sensor_msgs vision_msgs cv_bridge OpenCV)
target_link_libraries(red_target_detector ${OpenCV_LIBS})

install(TARGETS target_mover red_target_detector
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS eval/evaluate.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY worlds config launch DESTINATION share/${PROJECT_NAME})

ament_package()
