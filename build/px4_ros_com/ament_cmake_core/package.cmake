set(_AMENT_PACKAGE_NAME "px4_ros_com")
set(px4_ros_com_VERSION "0.1.0")
set(px4_ros_com_MAINTAINER "Beniamino Pozzan <beniamino.pozzan@gmail.com>")
set(px4_ros_com_BUILD_DEPENDS "eigen" "ros_environment" "builtin_interfaces" "rclcpp" "px4_msgs" "geometry_msgs" "sensor_msgs" "std_msgs" "launch" "launch_testing" "launch_testing_ros")
set(px4_ros_com_BUILDTOOL_DEPENDS "ament_cmake" "eigen3_cmake_module")
set(px4_ros_com_BUILD_EXPORT_DEPENDS "eigen" "builtin_interfaces" "rclcpp" "px4_msgs" "geometry_msgs" "sensor_msgs" "std_msgs" "launch" "launch_testing" "launch_testing_ros")
set(px4_ros_com_BUILDTOOL_EXPORT_DEPENDS "eigen3_cmake_module")
set(px4_ros_com_EXEC_DEPENDS "rosidl_default_runtime" "ros2launch" "builtin_interfaces" "rclcpp" "px4_msgs" "geometry_msgs" "sensor_msgs" "std_msgs" "launch" "launch_testing" "launch_testing_ros")
set(px4_ros_com_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(px4_ros_com_GROUP_DEPENDS )
set(px4_ros_com_MEMBER_OF_GROUPS )
set(px4_ros_com_DEPRECATED "")
set(px4_ros_com_EXPORT_TAGS)
list(APPEND px4_ros_com_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
