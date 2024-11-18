#pragma once

#include <QMainWindow>
#include <QGraphicsScene>

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
    QGraphicsScene *videoScene; // Scene for video rendering

    void setupConnections(); // Function to handle signal-slot connections

private slots:
    void addWaypoint();       // Slot to handle waypoint addition
    void startRecording();    // Slot for "Start Recording" button
    void stopRecording();     // Slot for "Stop Recording" button
    void stopFlight();        // Slot for "Stop Flight" button
    void updateCoordinates(); // Slot to update drone's coordinates
};