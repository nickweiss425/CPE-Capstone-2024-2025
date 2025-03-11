#include "lorasubscriber.hpp"
#include "serialstructs.hpp"

#define PREAMBLESIZE 2


/* Node entry point */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoraSubscriber>());
    rclcpp::shutdown();
    return 0;
}


/* Lora subscriber constructor that initializes the node subscribers and serial port */
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
    heartbeat_ping_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
          "/heartbeat/ping", 10, std::bind(&LoraSubscriber::heartbeat_ping_topic_callback, this, _1));
    heartbeat_ack_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
          "/heartbeat/ack", 10, std::bind(&LoraSubscriber::heartbeat_ack_topic_callback, this, _1));
    flight_command_subscription_ = this->create_subscription<gui_messages::msg::FlightCommand>(
          "flight_command", 10, std::bind(&LoraSubscriber::flight_command_topic_callback, this, _1));
    waypoint_ack_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
          "/waypoint_ack", 10, std::bind(&LoraSubscriber::waypoint_ack_topic_callback, this, _1));

    /* init serial port */
    try {
        serial_.Open(path);
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what()); 
    }
}


/* Close the serial connection */
LoraSubscriber::~LoraSubscriber() {
    serial_.Close();
}


/* Receive ROS GPS data and publish to serial */
void LoraSubscriber::gps_topic_callback(const sensor_msgs::msg::NavSatFix &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling gps"); 
    gps_serialized_t gps;
    gps.latitude = msg.latitude;
    gps.longitude = msg.longitude;
    gps.altitude = msg.altitude;
    static char preamble[PREAMBLESIZE + 1] = ":)";

    /* create byte array to hold data */
    uint8_t buffer[sizeof(gps_serialized_t) + PREAMBLESIZE];
    std::memcpy(buffer, preamble, PREAMBLESIZE);
    std::memcpy(buffer + PREAMBLESIZE, &gps, sizeof(gps_serialized_t));
    /* write to serial port byte by byte*/
    for (size_t i = 0; i < sizeof(gps_serialized_t) + PREAMBLESIZE; i++)
    {
        serial_ << buffer[i];
    }
}


/* Receive ROS IMU data and publish to serial */
void LoraSubscriber::imu_topic_callback(const sensor_msgs::msg::Imu &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling imu"); 
    imu_serialized_t imu;
    imu.orientation = msg.orientation;
    imu.angular_velocity = msg.angular_velocity;
    imu.linear_acceleration = msg.linear_acceleration;
    static char preamble[PREAMBLESIZE + 1] = ":)";

    /* create byte array to hold data */
    uint8_t buffer[sizeof(imu_serialized_t) + PREAMBLESIZE];
    std::memcpy(buffer, preamble, PREAMBLESIZE);
    std::memcpy(buffer + PREAMBLESIZE, &imu, sizeof(imu_serialized_t));

    /* write to serial port byte by byte*/
    for (size_t i = 0; i < sizeof(imu_serialized_t) + PREAMBLESIZE; i++)
    {
        serial_ << buffer[i];
    }
}


/* Receive ROS STATE data and publish to serial */
void LoraSubscriber::flight_state_topic_callback(const std_msgs::msg::Int32 &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling flight state"); 
    flight_state_serialized_t fs; 
    fs.state = msg.data;
    static char preamble[PREAMBLESIZE + 1] = ":)";

    /* create byte array to hold data */
    uint8_t buffer[sizeof(flight_state_serialized_t) + PREAMBLESIZE];
    std::memcpy(buffer, preamble, PREAMBLESIZE);
    std::memcpy(buffer + PREAMBLESIZE, &fs, sizeof(flight_state_serialized_t));

    /* write to serial port byte by byte*/
    for (size_t i = 0; i < sizeof(flight_state_serialized_t) + PREAMBLESIZE; i++)
    {
        serial_ << buffer[i];
    }
}


/* Receive ROS HEARTBEAT PING message and publish to serial */
void LoraSubscriber::heartbeat_ping_topic_callback(const std_msgs::msg::Empty &msg)
{
    (void)msg;

    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling heartbeat ping");
    heartbeat_ping_serialized_t hs;
    static char preamble[PREAMBLESIZE + 1] = ":)";

    /* create byte array to hold data */
    uint8_t buffer[sizeof(heartbeat_ping_serialized_t) + PREAMBLESIZE];
    std::memcpy(buffer, preamble, PREAMBLESIZE);
    std::memcpy(buffer + PREAMBLESIZE, &hs, sizeof(heartbeat_ping_serialized_t));

    /* write to serial port byte by byte*/
    for (size_t i = 0; i < sizeof(heartbeat_ping_serialized_t) + PREAMBLESIZE; i++)
    {
        serial_ << buffer[i];
    }
}


/* Receive ROS HEARTBEAT ACK message and publish to serial */
void LoraSubscriber::heartbeat_ack_topic_callback(const std_msgs::msg::Empty &msg)
{
    (void)msg;

    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling heartbeat ack");
    heartbeat_ack_serialized_t hs;
    static char preamble[PREAMBLESIZE + 1] = ":)";

    /* create byte array to hold data */
    uint8_t buffer[sizeof(heartbeat_ack_serialized_t) + PREAMBLESIZE];
    std::memcpy(buffer, preamble, PREAMBLESIZE);
    std::memcpy(buffer + PREAMBLESIZE, &hs, sizeof(heartbeat_ack_serialized_t));

    /* write to serial port byte by byte*/
    for (size_t i = 0; i < sizeof(heartbeat_ack_serialized_t) + PREAMBLESIZE; i++)
    {
        serial_ << buffer[i];
    }
}


/* Receive ROS STATE data and publish to serial */
void LoraSubscriber::flight_command_topic_callback(const gui_messages::msg::FlightCommand &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling flight command");
    flight_command_serialized_t fc;
    fc.latitude_deg = msg.latitude_deg;
    fc.longitude_deg = msg.longitude_deg;
    fc.altitude = msg.altitude;
    fc.radius = msg.radius;
    fc.length = msg.length;
    fc.duration = msg.duration;
    fc.waypoint_type = msg.waypoint_type;
    static char preamble[PREAMBLESIZE + 1] = ":)";

    /* create byte array to hold data */
    uint8_t buffer[sizeof(flight_command_serialized_t) + PREAMBLESIZE];
    std::memcpy(buffer, preamble, PREAMBLESIZE);
    std::memcpy(buffer + PREAMBLESIZE, &fc, sizeof(flight_command_serialized_t));

    /* write to serial port byte by byte*/
    for (size_t i = 0; i < sizeof(flight_command_serialized_t) + PREAMBLESIZE; i++)
    {
        serial_ << buffer[i];
    }
}


/* Receive WAYPOINT ACK data and publish to serial */
void LoraSubscriber::waypoint_ack_topic_callback(const std_msgs::msg::Int32 &msg)
{
    RCLCPP_INFO(this->get_logger(), "LORA subscriber handling waypoint ack");
    waypoint_ack_serialized_t wa;
    wa.ack = msg.data;
    static char preamble[PREAMBLESIZE + 1] = ":)";

    /* create byte array to hold data */
    uint8_t buffer[sizeof(waypoint_ack_serialized_t) + PREAMBLESIZE];
    std::memcpy(buffer, preamble, PREAMBLESIZE);
    std::memcpy(buffer + PREAMBLESIZE, &wa, sizeof(waypoint_ack_serialized_t));

    /* write to serial port byte by byte*/
    for (size_t i = 0; i < sizeof(waypoint_ack_serialized_t) + PREAMBLESIZE; i++)
    {
        serial_ << buffer[i];
    }
}
