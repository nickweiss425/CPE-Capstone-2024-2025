#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/flight_command.hpp>
#include <iostream>

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher(int state) : Node("command_publisher")
    {
        // Create a publisher that publishes to the "desired_state" topic
        publisher_ = this->create_publisher<custom_msgs::msg::FlightCommand>("flight_command", 10);

        // Publish a single message
        custom_msgs::msg::FlightCommand msg;    
        msg.latitude_deg = 35.272;
        msg.longitude_deg = -120.681;
        msg.altitude = -10.0;
        msg.radius = 10.0;
        msg.length = 10.0;
        msg.duration = 10.0;
        msg.waypoint_type = state;

        //RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.data);
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<custom_msgs::msg::FlightCommand>::SharedPtr publisher_;
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
    auto node = std::make_shared<CommandPublisher>(state);
    rclcpp::shutdown();
    return 0;
}
