#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class VideoPublisher : public rclcpp::Node
{
  public:
    VideoPublisher()
    : Node("video_publisher")
    {
        // Create a standard publisher for sensor_msgs::msg::Image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

        // Open the webcam (device 0 by default)
        /*
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open the webcam");
            return;
        }
        */

        // Timer to capture and publish images every 30ms
        timer_ = this->create_wall_timer(
            30ms, std::bind(&VideoPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        cv::Mat frame;
        frame = cv::imread("jelly.png"); 
        // cap_ >> frame;  // Capture a new frame

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture an image");
            return;
        }

        // Convert OpenCV Mat to ROS Image message using cv_bridge
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publish the image directly
        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  return 0;
}
