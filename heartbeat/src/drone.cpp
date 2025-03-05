#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Drone : public rclcpp::Node
{
public:
    Drone()
    : Node("drone")
    {
        /* Create subscriber to receive ping from ground station */
        subscription_ = this->create_subscription<std_msgs::msg::Empty>(
            "/heartbeat/ping", 10, std::bind(&Drone::ping_callback, this, _1));
        /* Create publisher to send ack to the ground station */
        publisher_ = this->create_publisher<std_msgs::msg::Empty>("/heartbeat/ack", 10);
        /* Create publisher to land the drone */
        land_publisher_ = this->create_publisher<std_msgs::msg::Int32>("desired_state", 10);
    }

private:
    /* Callback function to receive ping */
    void ping_callback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        RCLCPP_INFO(this->get_logger(), "Ping received");
        /* Send ack */
        auto ack_msg = std_msgs::msg::Empty();
        publisher_->publish(ack_msg);
        /* Reset timer */
        start_timer();
    }

    /* Start/restart timer for monitoring pings */
    void start_timer()
    {
        if (timer_)
        {
            timer_->cancel();
        }
        timer_ = this->create_wall_timer(
            10000ms, std::bind(&Drone::timer_callback, this));
    }

    /* Timeout */
    void timer_callback()
    {
        std_msgs::msg::Int32 msg;
        msg.data = 9; /* land in place */
        land_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Did not receive ping");
        timer_->cancel();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr land_publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Drone>());
    rclcpp::shutdown();
    return 0;
}

