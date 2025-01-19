cmake_minimum_required(VERSION 3.8)
project(lora)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Manually link libserial
find_library(SERIAL_LIB serial)

include_directories(include)

add_executable(listener src/subscriber.cpp)
target_link_libraries(listener ${SERIAL_LIB})
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

