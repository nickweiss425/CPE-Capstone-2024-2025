#pragma once
#pragma pack(push, 1)

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#define MAX_SIZE 64

enum topic_st {
    GPS,
    IMU,
    DESIRED_STATE
};

struct gps_serialized_t {
    topic_st topic = GPS;
    double latitude;
    double longitude; 
    double altitude;
};

struct imu_serialized_t {
    topic_st topic = IMU;
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Vector3 angular_velocity;
    geometry_msgs::msg::Vector3 linear_acceleration;
};

struct flight_state_serialized_t {
    topic_st topic = DESIRED_STATE;
    uint32_t state;
};

#pragma pack(pop)

