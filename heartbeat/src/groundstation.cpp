#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class GroundStation : public rclcpp::Node
{
public:
    GroundStation()
    : Node("groundstation")
    {
        /* Create publisher to send ping to the drone */
        publisher_ = this->create_publisher<std_msgs::msg::Empty>("/heartbeat/ping", 10);
        /* Create publisher to indicate to GUI connection status  */
        status_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/heartbeat/disconnected", 10);
        /* Create subscriber to receive ack from  drone */
        subscription_ = this->create_subscription<std_msgs::msg::Empty>(
            "/heartbeat/ack", 10, std::bind(&GroundStation::ack_callback, this, _1));
        /* Create timer to send pings at reguler interval */
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&GroundStation::timer_callback, this));
        /* Monitor if an ack was received */
        received_ack_ = true;
    }

private:
    /* Send ping */
    void timer_callback()
    {
        auto msg = std_msgs::msg::Empty();

        /* Check if we received an ack since the last ping */
        if (!received_ack_)
        {
            RCLCPP_INFO(this->get_logger(), "Did not recieve ACK");
            status_publisher_->publish(msg);
        }

        RCLCPP_INFO(this->get_logger(), "Sending ping");
        publisher_->publish(msg);

        received_ack_ = false;
    }

    /* Recieve ack */
    void ack_callback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        RCLCPP_INFO(this->get_logger(), "Ack received");
        received_ack_ = true;
    }

    bool received_ack_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr status_publisher_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundStation>());
    rclcpp::shutdown();
    return 0;
}

