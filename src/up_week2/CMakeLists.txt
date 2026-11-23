cmake_minimum_required(VERSION 3.8)
project(up_week2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(includes)

install(DIRECTORY worlds launch config eval DESTINATION share/${PROJECT_NAME})

ament_package()
