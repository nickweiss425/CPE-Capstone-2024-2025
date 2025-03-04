#include "waypointmanager.hpp"

/**
 * @brief Construct a new WaypointManager object for waypoint storage and communication
 */
WaypointManager::WaypointManager(QObject *parent) : QObject(parent)
{
}

/**
 * @brief Returns a QVariantList containing the waypoints.
 *
 * This function returns a QVariantList containing the waypoints. Each waypoint is represented as a QVariantMap
 * with the following keys:
 * - "coordinate": The coordinate of the waypoint.
 * - "radius": The radius of the waypoint.
 * - "altitude": The altitude of the waypoint.
 * - "duration": The duration of the waypoint.
 * - "type": The type of the waypoint.
 *
 * @return A QVariantList containing the waypoints.
 */
QVariantList WaypointManager::waypoints() const {
    QVariantList list;
    for (const auto &waypoint : m_waypoints) {
        QVariantMap map;
        map["coordinate"] = QVariant::fromValue(waypoint.coordinate);
        map["radius"] = waypoint.radius;
        map["altitude"] = waypoint.altitude;
        map["duration"] = waypoint.duration;
        map["type"] = static_cast<int>(waypoint.type);
        list.append(map);
    }
    return list;
}

/**
 * @brief Adds a new waypoint to the waypoint manager from coords and provides default values for the waypoint attributes.
 * 
 * This function creates a new waypoint with the given latitude and longitude coordinates
 * and adds it to the list of waypoints managed by the WaypointManager.
 * 
 * @param latitude The latitude coordinate of the new waypoint.
 * @param longitude The longitude coordinate of the new waypoint.
 */
void WaypointManager::onWaypointAdded(double latitude, double longitude) {
    Waypoint waypoint;
    waypoint.coordinate = QGeoCoordinate(latitude, longitude);
    waypoint.radius = 0;
    waypoint.altitude = 0;
    waypoint.duration = 0;
    waypoint.type = UNINITIALIZED;
    m_waypoints.append(waypoint);
    emit resetWaypointButtons();
}

/**
 * @brief Handles the selection of a waypoint.
 * 
 * This function is called when a waypoint is selected on the map
 * It emits a signal to retrieve the attributes of the selected waypoint and updates the selected index.
 * If the index is out of range or invalid, it emits a signal with default attribute values, which means a right click not on a waypoint.
 * 
 * @param index The index of the selected waypoint.
 */
void WaypointManager::onWaypointSelected(int index) {
    if (index == -1 || index >= m_waypoints.size()) {
        emit getWaypointAttributes(0, 0, 0, WaypointType::DEFAULT);
        return;
    }
    m_selectedIndex = index;
    emit getWaypointAttributes(m_waypoints[index].radius, m_waypoints[index].altitude, m_waypoints[index].duration, static_cast<int>(m_waypoints[index].type));
}

/**
 * @brief Removes a waypoint at the specified index.
 *
 * This function removes a waypoint from the `m_waypoints` container at the given index.
 * If the index is out of range, i.e., less than 0 or greater than or equal to the size of `m_waypoints`,
 * the function does nothing.
 *
 * @param index The index of the waypoint to be removed.
 */
void WaypointManager::onWaypointRemoved(int index) {
    if (index < 0 || index >= m_waypoints.size()) {
        return;
    }
    m_waypoints.remove(index);
}

/**
 * @brief Updates the attributes of a selected waypoint based on the GUI input.
 * 
 * This function updates the radius, altitude, duration, and type of a selected waypoint
 * in the waypoint manager.
 * 
 * @param radius The new radius of the waypoint.
 * @param altitude The new altitude of the waypoint.
 * @param duration The new duration of the waypoint.
 * @param type The new type of the waypoint.
 */
void WaypointManager::updateWaypointAttributes(double radius, double altitude, double duration, int type) {
    if (m_selectedIndex < 0 || m_selectedIndex >= m_waypoints.size()) {
        return;
    }

    // Invalid type
    if (type == 5 || type == -1) {
        return;
    }

    // Invalid data
    if (radius <= 0 || altitude <= 0 || duration <= 0) {
        return;
    }

    m_waypoints[m_selectedIndex].radius = radius;
    m_waypoints[m_selectedIndex].altitude = altitude;
    m_waypoints[m_selectedIndex].duration = duration;
    m_waypoints[m_selectedIndex].type = static_cast<WaypointType>(type);
    emit updateWaypointVisual(m_selectedIndex, type);
}

/**
 * @brief Updates the map's drone position with the given latitude and longitude.
 * 
 * This function emits the updateDronePosition signal to notify the map about the new drone position.
 * 
 * @param latitude The latitude of the drone's position.
 * @param longitude The longitude of the drone's position.
 */
void WaypointManager::getDronePosition(double latitude, double longitude) {
    emit updateDronePosition(latitude, longitude);
}

const Waypoint &WaypointManager::getNextWaypoint() {
    if (m_waypoints.empty()) {
        throw std::runtime_error("No waypoints available");
    }
    return m_waypoints.front();
}

gui_messages::msg::FlightCommand WaypointManager::getFlightCommand(const Waypoint &waypoint) {
    gui_messages::msg::FlightCommand flightCommand;
    flightCommand.latitude_deg = waypoint.coordinate.latitude();
    flightCommand.longitude_deg = waypoint.coordinate.longitude();
    flightCommand.altitude = waypoint.altitude;
    flightCommand.duration = waypoint.duration;
    flightCommand.waypoint_type = static_cast<int>(waypoint.type);

    switch (waypoint.type) {
        case WaypointType::CIRCLE:
        case WaypointType::FIGUREEIGHT:
            flightCommand.radius = waypoint.radius;
            break;
        case WaypointType::SQUARE:
            flightCommand.length = waypoint.radius;
        default:
            break;
    }

    return flightCommand;
}

/**
 * @brief Handles the reception of drone state messages.
 *
 * This function is called when a drone state message is received. It converts the received
 * message data to a FlightState enum and performs actions based on the type of message.
 *
 * @param msg The received drone state message.
 */
void WaypointManager::handleDroneStateReceive(const std_msgs::msg::Int32 &msg) {
    const int32_t state = static_cast<int32_t>(msg.data);
    switch (state) {
        default:
            break;
    }
}