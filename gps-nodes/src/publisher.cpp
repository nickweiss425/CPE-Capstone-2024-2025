#include "publisher.h"

MinimalPublisher::MinimalPublisher()
    : QObject(), Node("minimal_publisher"), count_(0), x_(5.0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    void MinimalPublisher::timer_callback()
    {
      auto message = sensor_msgs::msg::NavSatFix();

      // get sensor data here ?
      count_ += 0.001;
      x_ += 0.1;
      // Set some dummy data for the NavSatFix message
      message.latitude = 37.7749 + (count_ * 0.0001);  // Increment latitude slightly
      message.longitude = -122.4194 + (count_ * 0.0001); // Increment longitude slightly
      message.altitude = 10.0 + x_; // Dummy altitude

      // Set the status
      message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

      RCLCPP_INFO(this->get_logger(), "Publishing: Lat: '%f', Lon: '%f', Alt: '%f'",
		      message.latitude, message.longitude, message.altitude);
     
      publisher_->publish(message);
      emit coordinatesUpdated(message.latitude, message.longitude, message.altitude);
    }

