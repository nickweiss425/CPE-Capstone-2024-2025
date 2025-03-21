/* The publisher reads messages from LORA and re-publishes to ROS2 topics */
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

class LoraPublisher : public rclcpp::Node
{
public:
    LoraPublisher();
    ~LoraPublisher();

private:
    /* callback for reading and publishing */
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;

    /* publisher for each topic */
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr flight_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_ping_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_ack_publisher_;
    rclcpp::Publisher<gui_messages::msg::FlightCommand>::SharedPtr flight_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_ack_publisher_;

    /* libserial */
    LibSerial::SerialStream serial_;

    /* publishing functions */
    void publish_gps(uint8_t *raw_msg);
    void publish_imu(uint8_t *raw_msg);
    void publish_state(uint8_t *raw_msg);
    void publish_heartbeat_ping(uint8_t *raw_msg);
    void publish_heartbeat_ack(uint8_t *raw_msg);
    void publish_flight_command(uint8_t *raw_msg);
    void publish_waypoint_ack(uint8_t *raw_msg);
};

