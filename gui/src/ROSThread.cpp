#include "ROSThread.h"

/**
 * @brief Constructs a new ROSThread object.
 *
 * This constructor initializes a new ROSThread object with the given `node`.
 *
 * @param node A shared pointer to an instance of `rclcpp::Node`.
 */
ROSThread::ROSThread(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
}

/**
 * @brief Runs the ROSThread.
 *
 * This function is responsible for running the ROSThread. It uses the rclcpp::spin
 * function to start the ROS node and keep it spinning until the node is shutdown.
 */
void ROSThread::run()
{
     rclcpp::spin(node_);
}