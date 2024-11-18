cmake_minimum_required(VERSION 3.8)
project(flight_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2 Widgets QuickWidgets Location Positioning QmlImportScanner)

# Add the main.cpp file
set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
)

# Specify the UI file
set(UIS
    mainwindow.ui
)

# Wrap the UI file
qt5_wrap_ui(UI_HEADERS ${UIS})

# Add the main.h file for moc
set(HEADERS
    include/mainwindow.hpp
)

# Run moc on the header file
qt5_wrap_cpp(MOC_SOURCES ${HEADERS})

# Create executables
add_executable(${PROJECT_NAME} ${SOURCES} ${MOC_SOURCES})
#add_executable(talker src/publisher.cpp)
#add_executable(listener src/subscriber.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp Qt5 sensor_msgs)
#ament_target_dependencies(talker rclcpp sensor_msgs)
#ament_target_dependencies(listener rclcpp sensor_msgs)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Widgets
    Qt5::Quick
    Qt5::QuickWidgets
    Qt5::Location
    Qt5::Positioning
)

install(TARGETS
  ${PROJECT_NAME}
  #talker
  #listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()