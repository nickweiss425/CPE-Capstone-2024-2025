#pragma once

#include <QObject>
#include <QGeoCoordinate>
#include <QVariant>
#include <QVector>
#include <std_msgs/msg/int32.hpp>
#include "flightstates.hpp"

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
    double altitude;
    double duration;
    WaypointType type;
};

class WaypointManager : public QObject, flight_states
{
    Q_OBJECT

    Q_PROPERTY(QVariantList waypoints READ waypoints)

public:
    explicit WaypointManager(QObject *parent = nullptr);
    int selectedWaypointIndex() const { return m_selectedIndex; }

    QVariantList waypoints() const;

public slots:
    void onWaypointAdded(double latitude, double longitude);
    void onWaypointSelected(int index);
    void onWaypointRemoved(int index);
    void updateWaypointAttributes(double radius, double altitude, double duration, int type);
    void getDronePosition(double latitude, double longitude);
    void handleDroneStateReceive(const std_msgs::msg::Int32 &msg);
signals:
    void getWaypointAttributes(double radius, double altitude, double duration, int type);
    void updateDronePosition(double latitude, double longitude);

private:
    int m_selectedIndex = -1;
    QVector<Waypoint> m_waypoints;

    void onWaypointSelected(int index, const QGeoCoordinate &coordinate);
};