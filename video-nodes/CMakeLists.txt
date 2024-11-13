cmake_minimum_required(VERSION 3.8)
project(video-nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(talker src/video-publisher.cpp)
ament_target_dependencies(talker rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(listener src/video-subscriber.cpp)
ament_target_dependencies(listener rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
