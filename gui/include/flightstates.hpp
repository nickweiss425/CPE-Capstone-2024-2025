#pragma once

class flight_states {
public:
    enum class FlightState {
        LANDED = 0,
        TAKEOFF = 1,
        HOME_LAND = 2,
        CIRCLE_WAYPOINT = 3,
        CIRCLE_PATH = 4,
        WAYPOINT_HOLD = 5,
        SQUARE_WAYPOINT = 7,
        SQUARE_PATH = 8,
        LAND_IN_PLACE = 9,
        FIGURE8_WAYPOINT = 10,
        FIGURE8_PATH = 11
    };
    constexpr auto to_underlying(FlightState e) noexcept {
        return static_cast<std::underlying_type_t<FlightState>>(e);
    }
};