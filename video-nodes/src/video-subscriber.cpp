#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class VideoSubscriber : public rclcpp::Node
{
  public:
    VideoSubscriber()
    : Node("video_subscriber")
    {
        // Create a standard subscriber to subscribe to the "camera/image" topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&VideoSubscriber::image_callback, this, std::placeholders::_1));

        // Create opencv window
        cv::namedWindow("Received Image", cv::WINDOW_NORMAL); 
        cv::resizeWindow("Received Image", 800, 600);

    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert the ROS image message to an OpenCV image
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Display the image in an OpenCV window
            cv::imshow("Received Image", frame);
            cv::waitKey(1);  // Allow OpenCV to update the window
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoSubscriber>());
  rclcpp::shutdown();
  return 0;
}
