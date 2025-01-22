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

add_executable(subscriber src/lorasubscriber.cpp)
add_executable(publisher src/lorapublisher.cpp)
target_link_libraries(subscriber ${SERIAL_LIB})
target_link_libraries(publisher ${SERIAL_LIB})
ament_target_dependencies(subscriber rclcpp std_msgs sensor_msgs)
ament_target_dependencies(publisher rclcpp std_msgs sensor_msgs)

install(TARGETS
  subscriber
  publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()

