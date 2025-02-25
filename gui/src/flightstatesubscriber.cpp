#include "flightstatesubscriber.hpp"

using std::placeholders::_1;

StateSubscriber::StateSubscriber()
    : Node("state_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>("/waypoint_ack", 10, std::bind(&StateSubscriber::topic_callback, this, _1));
}

void StateSubscriber::topic_callback(const std_msgs::msg::Int32 &state) {
    RCLCPP_INFO(this->get_logger(), "Received flight state: %d", state.data);
    emit stateReceived(state);
}