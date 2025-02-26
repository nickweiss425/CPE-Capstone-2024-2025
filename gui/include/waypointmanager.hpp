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
    void onWaypointAdded(double latitude, double longitude); // Add a waypoint to the list from map signal
    void onWaypointSelected(int index); // Select a waypoint from the list from map signal
    void onWaypointRemoved(int index); // Remove a waypoint from the list from map signal
    void updateWaypointAttributes(double radius, double altitude, double duration, int type); // Update the attributes of a waypoint from the dialog
    void getDronePosition(double latitude, double longitude); // Get the drone's position from the GPSWidget
    void handleDroneStateReceive(const std_msgs::msg::Int32 &msg); // Handle the drone's state from incoming status messages
signals:
    void getWaypointAttributes(double radius, double altitude, double duration, int type); // Send the current attributes of a waypoint to the dialog
    void updateDronePosition(double latitude, double longitude); // Update the drone's position on the map
    void updateWaypointVisual(int index, int type); // Update the visual representation of a waypoint on the map
    void resetWaypointButtons(); // Reset waypoint buttons
private:
    int m_selectedIndex = -1;
    QVector<Waypoint> m_waypoints;
};