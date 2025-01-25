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
    /* get path argument */
    this->declare_parameter("path", "/dev/null");
    std::string path;
    this->get_parameter("path", path);

    /* init publishers */
    gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    flight_state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/desired_state", 10);

    /* init serial port */
    try {
        serial_.Open(path);
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&LoraPublisher::timer_callback, this));
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

    /* read message */
    uint8_t *msg = new uint8_t[MAX_SIZE];
    serial_ >> msg;
    topic_st *topic = (topic_st *)msg;
    switch(*topic)
    {
        case GPS:
            publish_gps(msg);
            break;
        case IMU:
            publish_imu(msg);
            break;
        case DESIRED_STATE:
            publish_state(msg);
            break;
        default:
            break;
    }

    delete[] msg;
}

void LoraPublisher::publish_gps(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received /gps/fix"); 

    gps_serialized_t *gps = (gps_serialized_t *)(raw_msg);
    auto msg = sensor_msgs::msg::NavSatFix();
    msg.latitude = gps->latitude;
    msg.longitude = gps->longitude;
    msg.altitude = gps->altitude;

    gps_publisher_->publish(msg);
}

void LoraPublisher::publish_imu(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received /imu"); 

    imu_serialized_t *imu = (imu_serialized_t *)(raw_msg);
    auto msg = sensor_msgs::msg::Imu();
    msg.orientation = imu->orientation;
    msg.angular_velocity = imu->angular_velocity;
    msg.linear_acceleration = imu->linear_acceleration;

    imu_publisher_->publish(msg);
}

void LoraPublisher::publish_state(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received /desired_state"); 

    flight_state_serialized_t *fs = (flight_state_serialized_t *)(raw_msg);
    auto msg = std_msgs::msg::Int32();
    msg.data = fs->state;

    flight_state_publisher_->publish(msg);
}

