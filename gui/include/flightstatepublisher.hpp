#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <QObject>
#include "std_msgs/msg/int32.hpp"
#include "flightstates.hpp"

class StatePublisher : public rclcpp::Node, public flight_states
{
public:
    StatePublisher();
    
    void publish_state(const FlightState &state);

private:
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};
