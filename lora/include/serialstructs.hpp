#pragma once
#pragma pack(push, 1)

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#define MAX_SIZE 128

enum class topic_st : uint8_t {
    GPS = 0,
    IMU = 1,
    DESIRED_STATE = 2,
    HEARTBEAT_PING = 3,
    HEARTBEAT_ACK = 4,
    FLIGHT_COMMAND = 5,
    WAYPOINT_ACK = 6
};

struct gps_serialized_t {
    topic_st topic = topic_st::GPS;
    double latitude;
    double longitude; 
    double altitude;
};

struct imu_serialized_t {
    topic_st topic = topic_st::IMU;
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Vector3 angular_velocity;
    geometry_msgs::msg::Vector3 linear_acceleration;
};

struct flight_state_serialized_t {
    topic_st topic = topic_st::DESIRED_STATE;
    uint32_t state;
};

struct heartbeat_ping_serialized_t {
    topic_st topic = topic_st::HEARTBEAT_PING;
};

struct heartbeat_ack_serialized_t {
    topic_st topic = topic_st::HEARTBEAT_ACK;
};

struct flight_command_serialized_t {
    topic_st topic = topic_st::FLIGHT_COMMAND;
    float latitude_deg;
    float longitude_deg;
    float altitude;
    float radius;
    float length;
    float duration;
    uint8_t waypoint_type;
};

struct waypoint_ack_serialized_t {
    int32_t ack;
};

#pragma pack(pop)

