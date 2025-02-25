#pragma once
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <QImage>
#include <QObject>
#include <opencv2/opencv.hpp>
#include "datalogger.hpp"

class VideoSubscriber : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    VideoSubscriber();
    void create_video();
    void end_video();

signals:
    void imageReceived(const QImage &image);

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::shared_ptr<DataLogger> dataLogger;
    cv::VideoWriter writer;
    int width, height, fps;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

