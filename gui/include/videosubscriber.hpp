#pragma once
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <QImage>
#include <QObject>

class VideoSubscriber : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    VideoSubscriber();

signals:
    void imageReceived(const QImage &image);

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};
