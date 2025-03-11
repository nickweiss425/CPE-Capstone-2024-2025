/* The subscriber subscribes to ROS2 topics, and forwards those data over LORA */
#pragma once

#include <memory>
#include "libserial/SerialStream.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "gui_messages/msg/flight_command.hpp"

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
    void heartbeat_ping_topic_callback(const std_msgs::msg::Empty &msg);
    void heartbeat_ack_topic_callback(const std_msgs::msg::Empty &msg);
    void flight_command_topic_callback(const gui_messages::msg::FlightCommand &msg);
    void waypoint_ack_topic_callback(const std_msgs::msg::Int32 &msg);

    /* subscription for each topic */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flight_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_ping_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_ack_subscription_;
    rclcpp::Subscription<gui_messages::msg::FlightCommand>::SharedPtr flight_command_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_ack_subscription_;

    /* libserial */
    LibSerial::SerialStream serial_;
};

