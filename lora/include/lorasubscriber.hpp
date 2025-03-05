/* The subscriber subscribes to ROS2 topics, and forwards those data over LORA */
#pragma once

#include <memory>
#include "libserial/SerialStream.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"

using std::placeholders::_1;

class LoraSubscriber : public rclcpp::Node
{
public:
    LoraSubscriber();
    ~LoraSubscriber();

private:
    /* callback for each topic */
    void gps_topic_callback(const sensor_msgs::msg::NavSatFix &msg);
    void imu_topic_callback(const sensor_msgs::msg::Imu &msg);
    void flight_state_topic_callback(const std_msgs::msg::Int32 &msg);
    void heartbeat_topic_callback(const std_msgs::msg::Empty &msg);

    /* subscription for each topic */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flight_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_subscription_;

    /* libserial */
    LibSerial::SerialStream serial_;
};

