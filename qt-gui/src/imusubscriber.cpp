#include "imusubscriber.hpp"

using std::placeholders::_1;

IMUSubscriber::IMUSubscriber()
    : Node("imu_subscriber") {
      subscription_ = this->create_subscription<sensor_msgs::msg::IMU>(
      "/imu", 10, std::bind(&IMUSubscriber::topic_callback, this, _1));
}

void IMUSubscriber::topic_callback(const sensor_msgs::msg:IMU &msg)
{
    RCLCPP_INFO(this->get_logger(), "subscribing to imu data");

    emit dataUpdated(
        msg.orientation_covariance,
        msg.angular_velocity_covariance,
        linear_acceleration_covariance
    );
}
