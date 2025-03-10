#pragma once
#include "gui_messages/msg/flight_command.hpp"

class flight_states {
public:
    // Enum class for the different flight states
    enum class FlightState {
        LANDED = 0,
        TAKEOFF = 1,
        HOME_LAND = 2,
        CIRCLE_WAYPOINT = 3,
        CIRCLE_PATH = 4,
        LOITER_WAYPOINT = 5,
        LOITER = 6,
        SQUARE_WAYPOINT = 7,
        SQUARE_PATH = 8,
        LAND_IN_PLACE = 9,
        FIGURE8_WAYPOINT = 10,
        FIGURE8_PATH = 11
    };

    // Function to convert the strongly typed enum class to its underlying type
    constexpr auto to_underlying(FlightState e) noexcept {
        return static_cast<std::underlying_type_t<FlightState>>(e);
    }
};