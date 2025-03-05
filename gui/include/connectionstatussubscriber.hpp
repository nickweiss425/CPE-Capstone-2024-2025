#pragma once
#include <chrono>
#include <memory>
#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class ConnectionStatusSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    ConnectionStatusSubscriber();

signals:
    void connectionStatusUpdate(int status);

private:
    void topic_callback(const std_msgs::msg::Int32 &msg);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

