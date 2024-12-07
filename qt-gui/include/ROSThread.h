#pragma once

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class ROSThread : public QThread
{
    Q_OBJECT

public:
    // Constructor that accepts a shared pointer to a ROS node
    ROSThread(std::shared_ptr<rclcpp::Node> node);

protected:
    // This method is called when the thread starts
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_; // ROS 2 node to spin
};