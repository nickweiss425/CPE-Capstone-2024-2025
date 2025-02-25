#include "gpssubscriber.hpp"

using std::placeholders::_1;

/**
 * @brief Constructor for GPSSubscriber class.
 * 
 * Initializes a GPSSubscriber object with a default node name and sets up a subscription to the "/gps/fix" topic.
 */
GPSSubscriber::GPSSubscriber()
    : Node("gps_subscriber") {
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
}

/**
 * @brief Callback function for the "/gps/fix" topic.
 * 
 * This function is called whenever a message is published to the "/gps/fix" topic. It extracts the latitude, longitude, and altitude from the message and emits a signal with these values.
 * 
 * @param msg The message published to the "/gps/fix" topic.
 */
void GPSSubscriber::topic_callback(const sensor_msgs::msg::NavSatFix &msg)
{
    RCLCPP_INFO(this->get_logger(), "Publishing: Lat: '%f', Lon: '%f', Alt: '%f'",
        msg.latitude, msg.longitude, msg.altitude);
    emit coordinatesUpdated(msg.latitude, msg.longitude, msg.altitude);
}