#include "waypointmanager.hpp"

WaypointManager::WaypointManager(QObject *parent) : QObject(parent)
{
}

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

void WaypointManager::onWaypointAdded(double latitude, double longitude) {
    Waypoint waypoint;
    waypoint.coordinate = QGeoCoordinate(latitude, longitude);
    waypoint.radius = 0;
    waypoint.altitude = 0;
    waypoint.duration = 0;
    waypoint.type = UNINITIALIZED;
    m_waypoints.append(waypoint);
}

void WaypointManager::onWaypointSelected(int index) {
    if (index == -1 || index >= m_waypoints.size()) {
        emit getWaypointAttributes(0, 0, 0, WaypointType::DEFAULT);
        return;
    }
    m_selectedIndex = index;
    emit getWaypointAttributes(m_waypoints[index].radius, m_waypoints[index].altitude, m_waypoints[index].duration, static_cast<int>(m_waypoints[index].type));
}

void WaypointManager::onWaypointRemoved(int index) {
    if (index < 0 || index >= m_waypoints.size()) {
        return;
    }
    m_waypoints.remove(index);
}

void WaypointManager::updateWaypointAttributes(double radius, double altitude, double duration, int type) {
    if (m_selectedIndex < 0 || m_selectedIndex >= m_waypoints.size()) {
        return;
    }
    m_waypoints[m_selectedIndex].radius = radius;
    m_waypoints[m_selectedIndex].altitude = altitude;
    m_waypoints[m_selectedIndex].duration = duration;
    m_waypoints[m_selectedIndex].type = static_cast<WaypointType>(type);
}

void WaypointManager::getDronePosition(double latitude, double longitude) {
    emit updateDronePosition(latitude, longitude);
}