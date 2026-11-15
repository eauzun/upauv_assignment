cmake_minimum_required(VERSION 3.8)
project(up_week1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(includes)

add_executable(heartbeat_publisher
  code/Publisher/main.cpp
  code/Publisher/HeartbeatPublisher.cpp)
ament_target_dependencies(heartbeat_publisher rclcpp std_msgs)

install(TARGETS heartbeat_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()
