#pragma once
#pragma pack(push, 1)

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

struct flight_state_serialized_t {
    const char *topic = "/desired_state";
    uint32_t state;
};

struct imu_serialized_t {
    const char *topic = "/imu";
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Vector3 angular_velocity;
    geometry_msgs::msg::Vector3 linear_acceleration;
};

struct gps_serialized_t {
    const char *topic = "/gps";
    double latitude;
    double longitude; 
    double altitude;
};

#pragma pack(pop)

