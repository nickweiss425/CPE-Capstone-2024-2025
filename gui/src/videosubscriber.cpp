#include <opencv2/opencv.hpp>
#include "videosubscriber.hpp"

/**
 * @brief Constructor for the VideoSubscriber class.
 * 
 * Initializes a VideoSubscriber object by setting the node name to "video_subscriber" and creating a subscription to the "camera/image" topic.
 * Also initializes the dataLogger instance and sets the width, height, and fps variables.
 */
VideoSubscriber::VideoSubscriber()
    : Node("video_subscriber")
{
    // Create a standard subscriber to subscribe to the "camera/image" topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image", 10, std::bind(&VideoSubscriber::image_callback, this,
        std::placeholders::_1));

    dataLogger = DataLogger::getInstance();

    width = 800;
    height = 600;
    fps = 5;
}

/**
 * @brief Callback function for processing image messages.
 *
 * This function is called whenever a new image message is received. It converts the ROS image message
 * to an OpenCV image and then resizes it. If recording is enabled, it writes the resized frame to a video file.
 * Otherwise, it closes the video file if it is open. Finally, it converts the resized frame to a QImage and emits
 * the imageReceived signal.
 *
 * @param msg The ROS image message to be processed.
 */
void VideoSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // Convert the ROS image message to an OpenCV image and then a QImage
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat resizedFrame;
        cv::resize(frame, resizedFrame, cv::Size(width, height));

        // if recording
        if (dataLogger->getRecording()) {
            // make sure video is open
            if (!writer.isOpened())
                create_video();
            writer.write(resizedFrame);
        }
        else {
            // make sure video is closed
            if (writer.isOpened())
                end_video();
        }
        
        QImage image(resizedFrame.data, resizedFrame.cols, resizedFrame.rows,
            resizedFrame.step, QImage::Format_BGR888);
        emit imageReceived(image.copy());
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
    }
}

/**
 * @brief Creates a video file with the current timestamp as the filename.
 * 
 * This function creates a video file with the current timestamp as the filename.
 * The video file is created using the OpenCV library and is encoded with the MJPG codec.
 * The video file is opened for writing and the video dimensions and frame rate are set.
 * If the video file fails to open, an error message is logged.
 */
void VideoSubscriber::create_video()
{
    auto now = this->get_clock()->now();
    std::time_t now_time_t = static_cast<std::time_t>(now.seconds()); 
    std::tm now_tm = *std::localtime(&now_time_t);
    std::ostringstream oss;
    oss << "output_" 
        << std::put_time(&now_tm, "%Y_%m_%d_%H_%M_%S")
        << ".avi";

    writer.open(oss.str(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps, cv::Size(width, height));

    if (!writer.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not create video");
    }
}

/**
 * @brief Ends the video playback and releases the writer.
 * 
 * This function is responsible for ending the video playback and releasing the writer object.
 * It should be called when the video playback is finished or when the application is closed.
 */
void VideoSubscriber::end_video()
{
    writer.release();
}

