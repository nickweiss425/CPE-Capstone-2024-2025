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

    void setupConnections(); // Function to handle signal-slot connections

private slots:
    void startRecording();    // Slot for "Start Recording" button
    void stopRecording();     // Slot for "Stop Recording" button
    void stopFlight();        // Slot for "Stop Flight" button
};