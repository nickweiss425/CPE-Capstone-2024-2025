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


/* Lora publisher constructor that initializes the node publishers and serial port */
LoraPublisher::LoraPublisher(): Node("lora_publisher") {
    /* get path argument */
    this->declare_parameter("path", "/dev/null");
    std::string path;
    this->get_parameter("path", path);

    /* init publishers */
    gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    flight_state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/desired_state", 10);
    heartbeat_ping_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/heartbeat/ping", 10);
    heartbeat_ack_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/heartbeat/ack", 10);
    flight_command_publisher_ = this->create_publisher<gui_messages::msg::FlightCommand>("flight_command", 10);
    waypoint_ack_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/waypoint_ack", 10);

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


/* close the serial connection */
LoraPublisher::~LoraPublisher() {
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
        case topic_st::HEARTBEAT_PING:
            total_bytes = sizeof(heartbeat_ping_serialized_t);
            RCLCPP_INFO(this->get_logger(), "Received HEARTBEAT PING message");
            break;
        case topic_st::HEARTBEAT_ACK:
            total_bytes = sizeof(heartbeat_ack_serialized_t);
            RCLCPP_INFO(this->get_logger(), "Received HEARTBEAT ACK message");
            break;
        case topic_st::FLIGHT_COMMAND:
            total_bytes = sizeof(flight_command_serialized_t);
            RCLCPP_INFO(this->get_logger(), "Received FLIGHT COMMAND message");
            break;
        case topic_st::WAYPOINT_ACK:
            total_bytes = sizeof(waypoint_ack_serialized_t);
            RCLCPP_INFO(this->get_logger(), "Received WAYPOINT ACK message");
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
            case topic_st::HEARTBEAT_PING:
                publish_heartbeat_ping(msg);
                break;
            case topic_st::HEARTBEAT_ACK:
                publish_heartbeat_ack(msg);
                break;
            case topic_st::FLIGHT_COMMAND:
                publish_flight_command(msg);
                break;
            case topic_st::WAYPOINT_ACK:
                publish_waypoint_ack(msg);
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


/* Publish received GPS data to ROS node */
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


/* Publish received IMU data to ROS node */
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


/* Publish received STATE data to ROS node */
void LoraPublisher::publish_state(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received /desired_state"); 

    flight_state_serialized_t *fs = (flight_state_serialized_t *)(raw_msg);
    auto msg = std_msgs::msg::Int32();
    msg.data = fs->state;

    flight_state_publisher_->publish(msg);
}


/* Publish received HEARTBEAT PING to ROS node */
void LoraPublisher::publish_heartbeat_ping(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received /heartbeat/ping");
    (void)raw_msg;

    auto msg = std_msgs::msg::Empty();
    heartbeat_ping_publisher_->publish(msg);
}


/* Publish received HEARTBEAT ACK to ROS node */
void LoraPublisher::publish_heartbeat_ack(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received /heartbeat/ack");
    (void)raw_msg;

    auto msg = std_msgs::msg::Empty();
    heartbeat_ack_publisher_->publish(msg);
}


/* Publish received FLIGHT_COMMAND data to ROS node */
void LoraPublisher::publish_flight_command(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received flight_command");

    flight_command_serialized_t *fc = (flight_command_serialized_t *)(raw_msg);
    auto msg = gui_messages::msg::FlightCommand();
    msg.latitude_deg = fc->latitude_deg;
    msg.longitude_deg = fc->longitude_deg;
    msg.altitude = fc->altitude;
    msg.radius = fc->radius;
    msg.length = fc->length;
    msg.duration = fc->duration;
    msg.waypoint_type = fc->waypoint_type;

    flight_command_publisher_->publish(msg);
}


/* Publish received WAYPOINT ACK data to ROS node */
void LoraPublisher::publish_waypoint_ack(uint8_t *raw_msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA received /waypoint_ack");

    waypoint_ack_serialized_t *wa = (waypoint_ack_serialized_t *)(raw_msg);
    auto msg = std_msgs::msg::Int32();
    msg.data = wa->ack;

    waypoint_ack_publisher_->publish(msg);
}
