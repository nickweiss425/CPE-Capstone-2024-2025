#pragma once

#include <memory>
#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    IMUSubscriber();

signals:
    void dataUpdated(
        const std::array<double, 9> orientation_covariance,
        const std::array<double, 9> angular_velocity_covariance,
        const std::array<double, 9> linear_acceleration_covariance
    );

private:
    void topic_callback(const sensor_msgs::msg::Imu &msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};
