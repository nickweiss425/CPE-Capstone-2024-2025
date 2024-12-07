#include "gpssubscriber.hpp"

using std::placeholders::_1;

GPSSubscriber::GPSSubscriber()
    : Node("gps_subscriber") {
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
    }

    void GPSSubscriber::topic_callback(const sensor_msgs::msg::NavSatFix &msg)
    {
	  RCLCPP_INFO(this->get_logger(), "Publishing: Lat: '%f', Lon: '%f', Alt: '%f'",
						msg.latitude, msg.longitude, msg.altitude);
    	  emit coordinatesUpdated(msg.latitude, msg.longitude, msg.altitude);
    }