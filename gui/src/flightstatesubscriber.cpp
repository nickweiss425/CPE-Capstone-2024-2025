#include "flightstatesubscriber.hpp"

using std::placeholders::_1;

/**
 * @brief Constructor for the StateSubscriber class.
 * 
 * Initializes a StateSubscriber object and sets the node name to "state_subscriber".
 * Creates a publisher that publishes to the "waypoint_ack" topic.
 */
StateSubscriber::StateSubscriber()
    : Node("state_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>("/waypoint_ack", 10, std::bind(&StateSubscriber::topic_callback, this, _1));
}

/**
 * @brief Callback function for the "waypoint_ack" topic.
 * 
 * This function is called whenever a message is received on the "waypoint_ack" topic.
 * 
 * @param state The received message.
 */
void StateSubscriber::topic_callback(const std_msgs::msg::Int32 &state) {
    RCLCPP_INFO(this->get_logger(), "Received flight state: %d", state.data);
    emit stateReceived(state);
}