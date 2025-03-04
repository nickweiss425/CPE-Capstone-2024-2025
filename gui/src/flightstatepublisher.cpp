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
    publisher_ = this->create_publisher<gui_messages::msg::FlightCommand>("flight_command", 10);
}

/**
 * @brief Publishes the flight state.
 * 
 * This function publishes the given flight state by converting it to an Int32 message
 * and publishing it using the publisher. It also logs the message if data logging is enabled.
 * 
 * @param state The flight state to be published.
 */
void StatePublisher::publish_state(const float &latitude, const float &longitude, const float &altitude, const float &radius, const float &length, const float &duration, const FlightState &state)
{
    gui_messages::msg::FlightCommand msg;
    msg.latitude_deg = latitude;
    msg.longitude_deg = longitude;
    msg.altitude = altitude;
    msg.radius = radius;
    msg.length = length;
    msg.duration = duration;
    msg.waypoint_type = to_underlying(state);
    
    // Log the message to see what is being sent
    RCLCPP_INFO(this->get_logger(), "Publishing state: '%d'", msg.waypoint_type);
    
    // Publish the message
    publisher_->publish(msg);

    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        QString commandText = QString("Flight state: %1, Latitude: %2, Longitude: %3, Altitude: %4, Radius: %5, Length: %6, Duration: %7")
            .arg(to_underlying(state))
            .arg(latitude)
            .arg(longitude)
            .arg(altitude)
            .arg(radius)
            .arg(length)
            .arg(duration);
        dataLogger_->log_data(commandText.toStdString());
    }
}
