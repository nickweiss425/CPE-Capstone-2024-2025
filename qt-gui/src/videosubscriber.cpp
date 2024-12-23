#include "videosubscriber.hpp"

VideoSubscriber::VideoSubscriber()
    : Node("video_subscriber")
    {
        // Create a standard subscriber to subscribe to the "camera/image" topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&VideoSubscriber::image_callback, this, std::placeholders::_1));
    }

void VideoSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert the ROS image message to an OpenCV image and then a QImage
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat resizedFrame;
            cv::resize(frame, resizedFrame, cv::Size(800, 600));
            
	        QImage image(resizedFrame.data, resizedFrame.cols, resizedFrame.rows, resizedFrame.step, QImage::Format_BGR888);
	        emit imageReceived(image.copy());
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }