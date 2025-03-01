#pragma once
#include <chrono>
#include <memory>
#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

class ConnectionStatusSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    ConnectionStatusSubscriber();

signals:
    void droneDisconnected();

private:
    void topic_callback(const std_msgs::msg::Empty &msg);
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};

