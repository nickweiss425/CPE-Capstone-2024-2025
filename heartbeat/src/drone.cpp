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
        /* Indicates if the drone has received the first ping yet */
        initiated_ = false;
    }

private:
    /* Callback function to receive ping */
    void ping_callback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Ping received");
        /* Send ack */
        auto ack_msg = std_msgs::msg::Empty();
        publisher_->publish(ack_msg);

        /* start measuring time between pings */
        if (!initiated_)
        {
            initiated_ = true;
            last_ping = std::chrono::steady_clock::now();
            return;
        }

        /* measure time difference */
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - last_ping).count();
        RCLCPP_INFO(this->get_logger(), "%d", delta);
        last_ping = now;
    }

    bool initiated_;
    std::chrono::steady_clock::time_point last_ping;
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

