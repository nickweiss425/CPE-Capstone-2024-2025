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

    // Validate the state value
    if (state.data < 0 || state.data > 11) {
        RCLCPP_WARN(this->get_logger(), "Received invalid flight state: %d", state.data);
    }

    emit stateReceived(state);                                                                                                                                                               

    RCLCPP_DEBUG(this->get_logger(), "Signal emitted for state: %d", state.data);
}