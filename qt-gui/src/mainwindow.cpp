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
    gpsWidget = new GPSWidget(ui->longitudeBox, ui->latitudeBox, ui->altitudeBox);

    // Set up the flight state publisher
    statePublisher = new StatePublisher();

    // Set up the video widget in the videoView
    ui->videoWidget->setObjectName("videoWidget");

    // Set default placeholder text for coordinate boxes
    ui->latitudeBox->setText("0.000000");
    ui->longitudeBox->setText("0.000000");
    ui->altitudeBox->setText("0.000000");

    // Create exit handler
    std::signal(SIGINT, MainWindow::signalHandler);

    // Connect buttons to their slots
    setupConnections();
}

MainWindow::~MainWindow() {
    delete ui;
    stopFlight();
    rclcpp::shutdown();
}

void MainWindow::setupConnections() {
    connect(ui->startFlightButton, &QPushButton::clicked, this, &MainWindow::startFlight);
    connect(ui->toggleRecordingButton, &QPushButton::clicked, this, &MainWindow::toggleRecording);
    connect(ui->stopFlightButton, &QPushButton::clicked, this, &MainWindow::stopFlight);
    initializeButtons();
}

void MainWindow::initializeButtons() {
    // Set buttons to their initial states (disabled except for start flight)
    ui->startFlightButton->setEnabled(true);
    ui->toggleRecordingButton->setEnabled(false);
    ui->stopFlightButton->setEnabled(false);

    ui->hoverControlButton1->setEnabled(false);
    ui->hoverControlButton2->setEnabled(false);
    ui->hoverControlButton3->setEnabled(false);
    ui->hoverControlButton4->setEnabled(false);
    ui->confirmHoverButton->setEnabled(false);
}

void MainWindow::unlockButtons() {
    // Enable buttons after start flight
    ui->startFlightButton->setEnabled(false);
    ui->toggleRecordingButton->setEnabled(true);
    ui->stopFlightButton->setEnabled(true);

    ui->hoverControlButton1->setEnabled(true);
    ui->hoverControlButton2->setEnabled(true);
    ui->hoverControlButton3->setEnabled(true);
    ui->hoverControlButton4->setEnabled(true);
    ui->confirmHoverButton->setEnabled(true);
}

void MainWindow::startFlight() {
    statePublisher->publish_state(TAKEOFF);
    unlockButtons();
}

void MainWindow::toggleRecording() {
    switch (recording) {
        case false:
            ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\271", nullptr));
            break;
        case true:
            ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
    }
    recording = !recording;
}

void MainWindow::stopFlight() {
    statePublisher->publish_state(LANDING);
    initializeButtons();
    recording = false;
    ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
}

void MainWindow::signalHandler(int) {
    rclcpp::shutdown();
    exit(0);
}