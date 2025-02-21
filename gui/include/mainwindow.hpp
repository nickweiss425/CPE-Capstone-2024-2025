#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include "gpswidget.hpp"
#include "flightstatepublisher.hpp"
#include "datalogger.hpp"
#include "videowidget.hpp"
#include "ROSThread.h"

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

private:
    Ui::MainWindow *ui;
    StatePublisher *statePublisher; // Publisher for flight state
    GPSWidget *gpsWidget;       // GPS widget for displaying coordinates
    IMUWidget *imuWidget;       // IMU widget for displaying imu data

    void setupConnections(); // Function to handle signal-slot connections
    void initializeButtons(); // Function to initialize button states
    void unlockButtons(); // Function to enable buttons after start flight
    void handleWaypointUpdate();
    void resetWaypointAttributes();
    static void signalHandler(int);     // Signal handler for SIGINT

signals:
    void setWaypointAttributes(double radius, double altitude, double duration, int type);

private slots:
    void startFlight();    // Slot for "Start Recording" button
    void toggleRecording();     // Slot for "Stop Recording" button
    void stopFlight();        // Slot for "Stop Flight" button
    void updateWaypointAttributes(double radius, double altitude, double duration, int type);
};