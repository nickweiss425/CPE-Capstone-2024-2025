/* The publisher reads messages from LORA and re-publishes to ROS2 topics */
#pragma once

#include <memory>
#include "libserial/SerialStream.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
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

    /* libserial */
    LibSerial::SerialStream serial_;

    /* publishing functions */
    void publish_gps(uint8_t *raw_msg);
    void publish_imu(uint8_t *raw_msg);
    void publish_state(uint8_t *raw_msg);
};

