#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <QObject>
#include "std_msgs/msg/int32.hpp"

typedef enum {
    LANDED = 0,
    TAKEOFF = 1,
    LANDING = 2
} FlightState;

class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher();
    
    void publish_state(const FlightState &state);

private:
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};
