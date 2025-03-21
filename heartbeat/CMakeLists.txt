cmake_minimum_required(VERSION 3.5)
project(heartbeat)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gui_messages REQUIRED)

add_executable(groundstation src/groundstation.cpp)
add_executable(drone src/drone.cpp)

ament_target_dependencies(groundstation rclcpp std_msgs)
ament_target_dependencies(drone rclcpp std_msgs gui_messages)

install(TARGETS
  groundstation
  drone
  DESTINATION lib/${PROJECT_NAME})

ament_package()

