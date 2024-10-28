#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::NavSatFix();

      // get sensor data here ?
      
      // Set some dummy data for the NavSatFix message
      message.latitude = 37.7749 + (count_ * 0.0001);  // Increment latitude slightly
      message.longitude = -122.4194 + (count_ * 0.0001); // Increment longitude slightly
      message.altitude = 10.0; // Dummy altitude

      // Set the status
      message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

      RCLCPP_INFO(this->get_logger(), "Publishing: Lat: '%f', Lon: '%f', Alt: '%f'",
		      message.latitude, message.longitude, message.altitude);
     
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

