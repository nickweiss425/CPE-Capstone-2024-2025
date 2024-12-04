#pragma once

#include <memory>
#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    GPSSubscriber();

signals:
    void coordinatesUpdated(double latitude, double longitude, double altitude);

private:
    void topic_callback(const sensor_msgs::msg::NavSatFix &msg);
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};
