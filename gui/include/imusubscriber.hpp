#pragma once

#include <memory>
#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class IMUSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    IMUSubscriber();

signals:
    void dataUpdated(
        const geometry_msgs::msg::Quaternion orientation,
        const geometry_msgs::msg::Vector3 angular_velocity,
        const geometry_msgs::msg::Vector3 linear_acceleration
    );

private:
    void topic_callback(const sensor_msgs::msg::Imu &msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};
