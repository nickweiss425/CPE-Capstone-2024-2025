#include "flightstatepublisher.hpp"

using namespace std::chrono_literals;

StatePublisher::StatePublisher() : Node("state_publisher")
{
    // Create a publisher that publishes to the "desired_state" topic
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("desired_state", 10);
}

void StatePublisher::publish_state(const FlightState &state)
{
    std_msgs::msg::Int32 msg;
    msg.data = state; // Set the state value
    
    // Log the message to see what is being sent
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.data);
    
    // Publish the message
    publisher_->publish(msg);
}
