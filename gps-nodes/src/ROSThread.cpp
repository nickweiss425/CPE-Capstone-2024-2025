#include "ROSThread.h"

ROSThread::ROSThread(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
}

void ROSThread::run()
{
     rclcpp::spin(node_);
}

