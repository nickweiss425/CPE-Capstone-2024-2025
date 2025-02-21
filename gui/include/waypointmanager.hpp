#pragma once

#include <QObject>
#include <QGeoCoordinate>
#include <QVariant>
#include <QVector>

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

class WaypointManager : public QObject
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
signals:
    void getWaypointAttributes(double radius, double altitude, double duration, int type);
    void updateDronePosition(const QGeoCoordinate &coordinate);

private:
    int m_selectedIndex = -1;
    QVector<Waypoint> m_waypoints;

    void onWaypointSelected(int index, const QGeoCoordinate &coordinate);
};