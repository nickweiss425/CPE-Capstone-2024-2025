#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;

class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher() : Node("state_publisher"), counter_(0)
    {
        // Create a publisher that publishes to the "desired_state" topic
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("desired_state", 10);

        // Timer callback to publish a message every second
        timer_ = this->create_wall_timer(1s, std::bind(&StatePublisher::publish_state, this));
    }

private:
    void publish_state()
    {
        std_msgs::msg::Int32 msg;
        msg.data = counter_;  // Set the state value

        // Log the message to see what is being sent
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.data);

        // Publish the message
        publisher_->publish(msg);

        // Update the counter for the next state
        counter_ = 1;
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();  // Shutdown ROS 2 when done
    return 0;
}
