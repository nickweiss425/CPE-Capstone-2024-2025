#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include "gpswidget.hpp"

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
    GPSWidget *gpsWidget;       // GPS widget for displaying coordinates
    bool recording = false;     // Flag to indicate if recording is in progress

    void setupConnections(); // Function to handle signal-slot connections
    void initializeButtons(); // Function to initialize button states
    void unlockButtons(); // Function to enable buttons after start flight
    static void signalHandler(int);     // Signal handler for SIGINT

private slots:
    void startFlight();    // Slot for "Start Recording" button
    void toggleRecording();     // Slot for "Stop Recording" button
    void stopFlight();        // Slot for "Stop Flight" button
};