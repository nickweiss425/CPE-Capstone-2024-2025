#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <iostream>

class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher(int state) : Node("state_publisher")
    {
        // Create a publisher that publishes to the "desired_state" topic
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("desired_state", 10);

        // Publish a single message
        std_msgs::msg::Int32 msg;
        msg.data = state;  // Set the desired state value
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.data);
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Ensure a command-line argument is provided
    if (argc < 2) {
        std::cerr << "Usage: ros2 run <package_name> <node_name> <state_value>\n";
        return 1;
    }

    // Convert argument to integer safely
    int state;
    try {
        state = std::stoi(argv[1]);
    } catch (const std::exception &e) {
        std::cerr << "Error: Invalid argument. Please provide an integer.\n";
        return 1;
    }

    // Create and run the node
    auto node = std::make_shared<StatePublisher>(state);
    rclcpp::shutdown();
    return 0;
}
