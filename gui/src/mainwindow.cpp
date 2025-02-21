#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QQuickWidget>
#include <QMediaPlayer>
#include <QVideoWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // Set up the GPS coordinates display
    gpsWidget = new GPSWidget(ui->longitudeBox, ui->latitudeBox);

    // Set up the IMU data stream
    imuWidget = new IMUWidget(ui->sensorData);

    // Set up the flight state publisher
    statePublisher = new StatePublisher();

    // Set up the video widget in the videoView
    ui->videoWidget->setObjectName("videoWidget");

    ui->sensorData->setText("");

    // Create exit handler
    std::signal(SIGINT, MainWindow::signalHandler);

    // Connect buttons to their slots
    setupConnections();
}

MainWindow::~MainWindow() {
    stopFlight();
    DataLogger::shutdown();
    delete ui;
    rclcpp::shutdown();
}

void MainWindow::setupConnections() {
    connect(ui->startFlightButton, &QPushButton::clicked, this, &MainWindow::startFlight);
    connect(ui->toggleRecordingButton, &QPushButton::clicked, this, &MainWindow::toggleRecording);
    connect(ui->stopFlightButton, &QPushButton::clicked, this, &MainWindow::stopFlight);
    connect(ui->confirmHoverButton, &QPushButton::clicked, this, &MainWindow::handleWaypointUpdate);
    connect(ui->waypointManager, &WaypointManager::getWaypointAttributes, this, &MainWindow::updateWaypointAttributes);
    connect(this, &MainWindow::setWaypointAttributes, ui->waypointManager, &WaypointManager::updateWaypointAttributes);

    QObject *map = ui->mapView->rootObject();
    if (map) {
        connect(map, SIGNAL(waypointAdded(double, double)), ui->waypointManager, SLOT(onWaypointAdded(double, double)));
        connect(map, SIGNAL(waypointSelected(int)), ui->waypointManager, SLOT(onWaypointSelected(int)));
        connect(map, SIGNAL(waypointRemoved(int)), ui->waypointManager, SLOT(onWaypointRemoved(int)));
        connect(gpsWidget, &GPSWidget::coordinatesUpdated, ui->waypointManager, &WaypointManager::getDronePosition);
        connect(ui->waypointManager, &WaypointManager::updateDronePosition, this, [this](double lat, double lon) {
            QMetaObject::invokeMethod(ui->mapView->rootObject(), "updateDronePosition",
                                    Q_ARG(QVariant, lat),
                                    Q_ARG(QVariant, lon));
        });
    }
    initializeButtons();
}

void MainWindow::initializeButtons() {
    // Set buttons to their initial states (control buttons and start flight are allowed)
    ui->startFlightButton->setEnabled(true);
    ui->toggleRecordingButton->setEnabled(false);
    ui->stopFlightButton->setEnabled(false);

    ui->hoverControlButton1->setEnabled(true);
    ui->hoverControlButton2->setEnabled(true);
    ui->hoverControlButton3->setEnabled(true);
    ui->hoverControlButton4->setEnabled(true);
    ui->confirmHoverButton->setEnabled(true);
}

void MainWindow::unlockButtons() {
    // Enable buttons after start flight
    ui->startFlightButton->setEnabled(false);
    ui->toggleRecordingButton->setEnabled(true);
    ui->stopFlightButton->setEnabled(true);
}

void MainWindow::startFlight() {
    statePublisher->publish_state(TAKEOFF);
    unlockButtons();
}

void MainWindow::toggleRecording() {
    auto dataLogger_ = DataLogger::getInstance();
    switch (dataLogger_->getRecording()) {
        case false:
            ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\271", nullptr));
            break;
        case true:
            ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
    }

    if (dataLogger_->getRecording()) {
        dataLogger_->close_file();
    } else {
        dataLogger_->create_file();
    }
    dataLogger_->switchRecording();
}

void MainWindow::stopFlight() {
    statePublisher->publish_state(LANDING);
    ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        dataLogger_->switchRecording();
    }
}

void MainWindow::updateWaypointAttributes(double radius, double altitude, double duration, int type) {
    // First reset all buttons to initial state
    ui->hoverControlButton1->setChecked(false);
    ui->hoverControlButton2->setChecked(false);
    ui->hoverControlButton3->setChecked(false);
    ui->hoverControlButton4->setChecked(false);

    // Clear text fields first
    ui->radiusBox->clear();
    ui->altitudeBox->clear();
    ui->durationBox->clear();

    // If no waypoint is selected (type == 5 or -1)
    if (type == 5 || type == -1) {
        ui->hoverControlButton1->setCheckable(false);
        ui->hoverControlButton2->setCheckable(false);
        ui->hoverControlButton3->setCheckable(false);
        ui->hoverControlButton4->setCheckable(false);
        ui->confirmHoverButton->setCheckable(false);
        resetWaypointAttributes();
        return;
    }

    // Enable all buttons for a valid selection
    ui->hoverControlButton1->setCheckable(true);
    ui->hoverControlButton2->setCheckable(true);
    ui->hoverControlButton3->setCheckable(true);
    ui->hoverControlButton4->setCheckable(true);
    ui->confirmHoverButton->setCheckable(true);

    // Set the text values if they exist
    if (radius > 0) ui->radiusBox->setText(QString::number(radius));
    if (altitude > 0) ui->altitudeBox->setText(QString::number(altitude));
    if (duration > 0) ui->durationBox->setText(QString::number(duration));

    // Set the appropriate button state
    if (type > 0 && type <= 4) {
        switch (type) {
            case 1:
                ui->hoverControlButton1->setChecked(true);
                break;
            case 2:
                ui->hoverControlButton2->setChecked(true);
                break;
            case 3:
                ui->hoverControlButton3->setChecked(true);
                break;
            case 4:
                ui->hoverControlButton4->setChecked(true);
                break;
        }
    }

    ui->hoverControlButton1->update();
    ui->hoverControlButton2->update();
    ui->hoverControlButton3->update();
    ui->hoverControlButton4->update();
    
}

void MainWindow::handleWaypointUpdate() {
    int waypointType = ui->hoverGroup->checkedId();
    emit setWaypointAttributes(ui->radiusBox->text().toDouble(), ui->altitudeBox->text().toDouble(), ui->durationBox->text().toDouble(), waypointType);
    
    resetWaypointAttributes();

    switch (waypointType) {
        case 1:
            ui->hoverControlButton1->setChecked(false);
            break;
        case 2:
            ui->hoverControlButton2->setChecked(false);
            break;
        case 3:
            ui->hoverControlButton3->setChecked(false);
            break;
        case 4:
            ui->hoverControlButton4->setChecked(false);
            break;
        default:
            break;
    }
}

void MainWindow::resetWaypointAttributes() {
    ui->hoverControlButton1->setCheckable(false);
    ui->hoverControlButton2->setCheckable(false);
    ui->hoverControlButton3->setCheckable(false);
    ui->hoverControlButton4->setCheckable(false);

    ui->hoverControlButton2->setChecked(false);
    ui->hoverControlButton1->setChecked(false);
    ui->hoverControlButton3->setChecked(false);
    ui->hoverControlButton4->setChecked(false);

    ui->hoverControlButton1->update();
    ui->hoverControlButton2->update();
    ui->hoverControlButton3->update();
    ui->hoverControlButton4->update();

    ui->confirmHoverButton->setChecked(false);
}

void MainWindow::signalHandler(int) {
    rclcpp::shutdown();
    exit(0);
}
