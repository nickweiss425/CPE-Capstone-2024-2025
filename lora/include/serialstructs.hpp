#pragma once
#pragma pack(push, 1)

struct flight_state_serialized {
    const char *topic = "desired_state";
    uint32_t state;
};

struct imu_serialized {
    const char *topic = "/imu";
};

struct gps_serialized {
    const char *topic = "/gps";
};

#pragma pack(pop)

