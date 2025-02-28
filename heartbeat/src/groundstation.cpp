#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;


class GroundStation : public rclcpp::Node
{
public:
    GroundStation()
    : Node("groundstation")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Empty>("/heartbeat/send", 10);
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&GroundStation::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Pinging");
        auto msg = std_msgs::msg::Empty();
        publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundStation>());
    rclcpp::shutdown();
    return 0;
}

