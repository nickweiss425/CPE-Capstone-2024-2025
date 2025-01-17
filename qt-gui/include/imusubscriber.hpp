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
    void dataUpdated(float64 orientation_covariance[9], float64 angular_velocity_covariance[9], float64 linear_acceleration_covariance[9]);

private:
    void topic_callback(const sensor_msgs::msg::IMU &msg);
    rclcpp::Subscription<sensor_msgs::msg::IMU>::SharedPtr subscription_;
};
