#include "flightstatepublisher.hpp"
#include "datalogger.hpp"

using namespace std::chrono_literals;

/**
 * @brief Constructor for the StatePublisher class.
 * 
 * Initializes a StatePublisher object and sets the node name to "state_publisher".
 * Creates a publisher that publishes to the "desired_state" topic.
 */
StatePublisher::StatePublisher() : Node("state_publisher")
{
    // Create a publisher that publishes to the "desired_state" topic
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("desired_state", 10);
}

/**
 * @brief Publishes the flight state.
 * 
 * This function publishes the given flight state by converting it to an Int32 message
 * and publishing it using the publisher. It also logs the message if data logging is enabled.
 * 
 * @param state The flight state to be published.
 */
void StatePublisher::publish_state(const FlightState &state)
{
    std_msgs::msg::Int32 msg;
    msg.data = to_underlying(state); // Set the state value
    
    // Log the message to see what is being sent
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.data);
    
    // Publish the message
    publisher_->publish(msg);

    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        QString flightStateText = QString("Flight State: %1\n") .arg(msg.data);
        dataLogger_->log_data(flightStateText.toStdString());
    }
}
