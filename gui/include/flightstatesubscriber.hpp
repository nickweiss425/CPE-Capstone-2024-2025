#pragma once
#include <chrono>
#include <memory>
#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "flightstates.hpp"

class StateSubscriber : public QObject, public rclcpp::Node, public flight_states {
    Q_OBJECT

public:
    StateSubscriber();

signals:
    void stateReceived(const std_msgs::msg::Int32 &msg);

private:
    void topic_callback(const std_msgs::msg::Int32 &state);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};