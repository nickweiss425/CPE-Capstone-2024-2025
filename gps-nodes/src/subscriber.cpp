#include "subscriber.h"

using std::placeholders::_1;

MinimalSubscriber::MinimalSubscriber()
       : QObject(), Node("minimal_subscriber") {
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

    void MinimalSubscriber::topic_callback(const sensor_msgs::msg::NavSatFix &msg)
    {
	  RCLCPP_INFO(this->get_logger(), "Publishing: Lat: '%f', Lon: '%f', Alt: '%f'",
						msg.latitude, msg.longitude, msg.altitude);
    	  emit coordinatesUpdated(msg.latitude, msg.longitude, msg.altitude);
    }

