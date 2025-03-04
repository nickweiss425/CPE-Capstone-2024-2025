cmake_minimum_required(VERSION 3.8)
project(flight_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTORCC ON)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gui_messages REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2 Widgets QuickWidgets Location Positioning QmlImportScanner Multimedia MultimediaWidgets)

# Add the main.cpp file
set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/videosubscriber.cpp
    src/videowidget.cpp
    src/ROSThread.cpp
    src/gpswidget.cpp
    src/gpssubscriber.cpp
    src/imuwidget.cpp
    src/imusubscriber.cpp
    src/connectionstatuswidget.cpp
    src/connectionstatussubscriber.cpp
    src/flightstatesubscriber.cpp
    src/flightstatepublisher.cpp
    src/datalogger.cpp
    src/waypointmanager.cpp
)

# Add the main.h file for moc
set(HEADERS
    include/mainwindow.hpp
    include/videosubscriber.hpp
    include/videowidget.hpp
    include/ROSThread.h
    include/gpswidget.hpp
    include/gpssubscriber.hpp
    include/imuwidget.hpp
    include/imusubscriber.hpp
    include/connectionstatuswidget.hpp
    include/connectionstatussubscriber.hpp
    include/flightstates.hpp
    include/flightstatesubscriber.hpp
    include/flightstatepublisher.hpp
    include/datalogger.hpp
    include/waypointmanager.hpp
)

# Run moc on the header file
qt5_wrap_cpp(MOC_SOURCES ${HEADERS})

# Create executables
add_executable(${PROJECT_NAME} ${SOURCES} ${MOC_SOURCES} resources.qrc)
ament_target_dependencies(${PROJECT_NAME} rclcpp Qt5 sensor_msgs geometry_msgs gui_messages)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE include gui_messages/msg)

target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Widgets
    Qt5::Quick
    Qt5::QuickWidgets
    Qt5::Location
    Qt5::Positioning
    Qt5::Multimedia
    Qt5::MultimediaWidgets
    cv_bridge::cv_bridge
    ${OpenCV_LIBS}
)

install(TARGETS
  ${PROJECT_NAME}
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
