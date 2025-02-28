#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using std::placeholders::_1;

class Drone : public rclcpp::Node
{
public:
    Drone()
    : Node("drone")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Empty>(
            "/heartbeat/send", 10, std::bind(&Drone::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::Empty::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Ping received");
    }
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Drone>());
    rclcpp::shutdown();
    return 0;
}

