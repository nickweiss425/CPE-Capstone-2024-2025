#include "connectionstatussubscriber.hpp"

using std::placeholders::_1;

ConnectionStatusSubscriber::ConnectionStatusSubscriber()
    : Node("connection_status_subscriber")
{
    subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "/heartbeat/disconnected", 10, std::bind(&ConnectionStatusSubscriber::topic_callback,
        this, _1));
}

void ConnectionStatusSubscriber::topic_callback(const std_msgs::msg::Empty &msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Lost connection with drone");
    emit droneDisconnected();
}

