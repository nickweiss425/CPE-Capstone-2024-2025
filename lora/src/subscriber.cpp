#include "subscriber.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoraSubscriber>());
    rclcpp::shutdown();
    return 0;
}

LoraSubscriber::LoraSubscriber(): Node("lora_subscriber") {
    /* init subscriptions */
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/gps/fix", 10, std::bind(&LoraSubscriber::gps_topic_callback, this, _1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/imu", 10, std::bind(&LoraSubscriber::imu_topic_callback, this, _1));
    flight_state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
          "desired_state", 10, std::bind(&LoraSubscriber::flight_state_topic_callback, this, _1));
}

void LoraSubscriber::gps_topic_callback(const sensor_msgs::msg::NavSatFix &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling gps"); 
}

void LoraSubscriber::imu_topic_callback(const sensor_msgs::msg::Imu &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling imu"); 
}

void LoraSubscriber::flight_state_topic_callback(const std_msgs::msg::Int32 &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling flight state"); 
}

