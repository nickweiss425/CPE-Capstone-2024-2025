#pragma once

#include <QObject>
#include <QGeoCoordinate>
#include <QVariant>
#include <QVector>
#include <std_msgs/msg/int32.hpp>
#include "rclcpp/rclcpp.hpp"
#include "flightstatepublisher.hpp"

enum WaypointType : int {
    UNINITIALIZED,
    LOITER,
    CIRCLE,
    FIGUREEIGHT,
    SQUARE,
    DEFAULT
};

struct Waypoint {
    QGeoCoordinate coordinate;
    double radius;
    double length;
    double altitude;
    double duration;
    WaypointType type;
};

class WaypointManager : public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariantList waypoints READ waypoints NOTIFY waypointsChanged)

public:
    explicit WaypointManager(QObject *parent = nullptr);
    QVariantList waypoints() const;
    const Waypoint getNextWaypoint();
    bool hasWaypoints() const { return !m_waypoints.isEmpty(); }
    gui_messages::msg::FlightCommand getFlightCommand(const Waypoint &waypoint);

signals:
    void waypointsChanged();
    void getWaypointAttributes(double radius, double altitude, double duration, int type);
    void updateWaypointVisual(int index, int type);
    void updateDronePosition(double latitude, double longitude);
    void resetWaypointButtons();
    void popWaypoint();

public slots:
    void onWaypointAdded(double latitude, double longitude);
    void onWaypointSelected(int index);
    void onWaypointRemoved(int index);
    void updateWaypointAttributes(double radius, double altitude, double duration, int type);
    void getDronePosition(double latitude, double longitude);

private:
    QVector<Waypoint> m_waypoints;
    int m_selectedIndex = -1;
};