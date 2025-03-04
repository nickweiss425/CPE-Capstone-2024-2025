#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <QObject>
#include "flightstates.hpp"

class StatePublisher : public rclcpp::Node, public flight_states
{
public:
    StatePublisher();
    
    void publish_state(const float &latitude, const float &longitude, const float &altitude, const float &radius, const float &length, const float &duration, const FlightState &state);

private:
    
    rclcpp::Publisher<gui_messages::msg::FlightCommand>::SharedPtr publisher_;
};
