#include "lorasubscriber.hpp"
#include "serialstructs.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoraSubscriber>());
    rclcpp::shutdown();
    return 0;
}

LoraSubscriber::LoraSubscriber(): Node("lora_subscriber") {
    /* get path argument */
    this->declare_parameter("path", "/dev/null");
    std::string path;
    this->get_parameter("path", path);

    /* init subscriptions */
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/gps/fix", 10, std::bind(&LoraSubscriber::gps_topic_callback, this, _1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/imu", 10, std::bind(&LoraSubscriber::imu_topic_callback, this, _1));
    flight_state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
          "desired_state", 10, std::bind(&LoraSubscriber::flight_state_topic_callback, this, _1));

    /* init serial port */
    try {
        serial_.Open(path);
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what()); 
    }
}

LoraSubscriber::~LoraSubscriber() {
    /* close the serial connection */
    serial_.Close();
}

void LoraSubscriber::gps_topic_callback(const sensor_msgs::msg::NavSatFix &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling gps"); 
    gps_serialized_t gps;
    gps.latitude = msg.latitude;
    gps.longitude = msg.longitude;
    gps.altitude = msg.altitude;
    serial_ << reinterpret_cast<uint8_t*>(&gps);
}

void LoraSubscriber::imu_topic_callback(const sensor_msgs::msg::Imu &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling imu"); 
    imu_serialized_t imu;
    imu.orientation = msg.orientation;
    imu.angular_velocity = msg.angular_velocity;
    imu.linear_acceleration = msg.linear_acceleration;
    serial_ << reinterpret_cast<uint8_t*>(&imu);
}

void LoraSubscriber::flight_state_topic_callback(const std_msgs::msg::Int32 &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling flight state"); 
    flight_state_serialized_t fs; 
    fs.state = msg.data;

    /* write to serial port */
    serial_ << reinterpret_cast<uint8_t*>(&fs);
}

