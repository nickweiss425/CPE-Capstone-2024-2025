#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QButtonGroup>
#include <QQuickWidget>
#include <QGeoCoordinate>
#include <csignal>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "gpswidget.hpp"
#include "imuwidget.hpp"
#include "connectionstatuswidget.hpp"
#include "flightstatepublisher.hpp"
#include "flightstatesubscriber.hpp"
#include "datalogger.hpp"
#include "waypointmanager.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void resetWaypointAttributes(); // Function to reset waypoint attributes on empty right click

private:
    Ui::MainWindow *ui;
    StatePublisher *statePublisher; // Publisher for flight state
    StateSubscriber *stateSubscriber; // Subscriber for flight state
    GPSWidget *gpsWidget;       // GPS widget for displaying coordinates
    IMUWidget *imuWidget;       // IMU widget for displaying imu data
    ConnectionStatusWidget *connectionStatusWidget; // Widget for displaying connection to drone

    int m_lastDelayDuration = 0; // Holds the necessary delay for the current waypoint's duration (ms)

    void setupConnections(); // Function to handle signal-slot connections
    void initializeButtons(); // Function to initialize button states
    void unlockButtons(); // Function to enable buttons after start flight
    void handleWaypointUpdate(); // Function to handle waypoint updates from waypointManager
    void handleDroneStateReceive(const std_msgs::msg::Int32 &msg); // Function to handle drone state updates from stateSubscriber
    flight_states::FlightState getFlightState(const Waypoint &waypoint); // Function to get the next flight state from a waypoint
    static void signalHandler(int); // Signal handler for SIGINT

signals:
    void setWaypointAttributes(double radius, double altitude, double duration, int type);

private slots:
    void startFlight(); // Slot for "Start Recording" button
    void toggleRecording(); // Slot for "Stop Recording" button
    void stopFlight(); // Slot for "Stop Flight" button
    void updateWaypointAttributes(double radius, double altitude, double duration, int type);
    void processNextWaypoint();
};
