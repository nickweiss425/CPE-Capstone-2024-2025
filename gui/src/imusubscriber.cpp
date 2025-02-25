#include "imusubscriber.hpp"

using std::placeholders::_1;

/**
 * @brief Constructor for the IMUSubscriber class.
 * 
 * Initializes the IMUSubscriber object by creating a subscription to the "/imu" topic with a queue size of 10.
 * The topic_callback function is bound to the subscription, which will be called whenever a new message is received.
 */
IMUSubscriber::IMUSubscriber()
    : Node("imu_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&IMUSubscriber::topic_callback, this, _1));
}

/**
 * @brief Callback function for the IMU topic.
 * 
 * This function is called whenever a new message is received on the IMU topic.
 * It emits a signal with the updated IMU data.
 * 
 * @param msg The received IMU message.
 */
void IMUSubscriber::topic_callback(const sensor_msgs::msg::Imu &msg)
{
    // RCLCPP_INFO(this->get_logger(), "subscribing to imu data");

    emit dataUpdated(
        msg.orientation,
        msg.angular_velocity,
        msg.linear_acceleration
    );
}
