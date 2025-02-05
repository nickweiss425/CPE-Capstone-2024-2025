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
            1000ms, std::bind(&LoraPublisher::timer_callback, this));
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
    size_t bytes_read = 0;
    char preamble_buf;
    static char preamble[3] = ":)"; 

    /* get one byte for topic */
    int first_preamble_read = 0;
    while (serial_.IsDataAvailable()) {
        serial_ >> preamble_buf;
        if (preamble_buf == preamble[0] && !first_preamble_read) {
            first_preamble_read = 1;
            continue;
        }
        if (preamble_buf == preamble[1] && first_preamble_read) {
            first_preamble_read = 0;
            break;
        }
    }
    if (serial_.IsDataAvailable())
    {
        serial_ >> msg[bytes_read++];
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No data available to read.");
        delete[] msg;
        return;
    }
    topic_st topic = (topic_st)msg[0];

    /* figure out how many bytes are left */
    size_t total_bytes = 0;
    switch(topic)
    {
        case topic_st::GPS:
	    total_bytes = sizeof(gps_serialized_t);
	    RCLCPP_INFO(this->get_logger(), "Received GPS message");
            break;
	case topic_st::IMU:
	    total_bytes = sizeof(imu_serialized_t);
	    RCLCPP_INFO(this->get_logger(), "Received IMU message");
            break;
	case topic_st::DESIRED_STATE:
	    total_bytes = sizeof(flight_state_serialized_t);
	    RCLCPP_INFO(this->get_logger(), "Received STATE message");
            break;
    default:
	    RCLCPP_INFO(this->get_logger(), "Received unknown message");
	    delete[] msg;
	    return;
    }
    
    /* read the rest */
    while (bytes_read < MAX_SIZE && bytes_read < total_bytes)
    {
        while (!serial_.IsDataAvailable());
        serial_ >> msg[bytes_read++];
    }

    RCLCPP_INFO(this->get_logger(), "Bytes read: %ld, Total bytes: %ld", bytes_read, total_bytes);

    if (bytes_read >= total_bytes)
    {
        switch (topic)
	{
	    case topic_st::GPS:
                publish_gps(msg);
                break;
	    case topic_st::IMU:
                publish_imu(msg);
                break;
	    case topic_st::DESIRED_STATE:
                publish_state(msg);
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Received unknown message topic.");
                break;
         }
    }
    else
    {
         RCLCPP_WARN(this->get_logger(), "Incomplete message received.");
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

