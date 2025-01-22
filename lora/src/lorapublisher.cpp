#include "lorapublisher.hpp"
#include "serialstructs.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoraPublisher>());
    rclcpp::shutdown();
    return 0;
}

LoraPublisher::LoraPublisher(): Node("lora_publisher") {
    /* init publishers */
    gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    flight_state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/desired_state", 10);

    /* init serial port */
    try {
        serial_.Open("/dev/ttyACM0");
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&LoraPublisher::timer_callback, this));
    }
    catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what()); 
    }
}

LoraPublisher::~LoraPublisher() {
    /* close the serial connection */
    serial_.Close();
}

/* read from serial port and reconstruct message */
void LoraPublisher::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "LORA timer callback"); 
}

