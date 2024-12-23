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

    // Set up the video widget in the videoView
    ui->videoWidget->setObjectName("videoWidget");

    // Set default placeholder text for coordinate boxes
    ui->latitudeBox->setText("0.000000");
    ui->longitudeBox->setText("0.000000");
    ui->altitudeBox->setText("0.000000");

    // Connect buttons to their slots
    setupConnections();
}

MainWindow::~MainWindow() {
    delete ui;
    rclcpp::shutdown();
}

void MainWindow::setupConnections() {
    connect(ui->startRecordingButton, &QPushButton::clicked, this, &MainWindow::startRecording);
    connect(ui->stopRecordingButton, &QPushButton::clicked, this, &MainWindow::stopRecording);
    connect(ui->stopFlightButton, &QPushButton::clicked, this, &MainWindow::stopFlight);
}

void MainWindow::startRecording() {
    QMessageBox::information(this, "Start Recording", "Recording started.");
}

void MainWindow::stopRecording() {
    QMessageBox::information(this, "Stop Recording", "Recording stopped.");
}

void MainWindow::stopFlight() {
    QMessageBox::warning(this, "Stop Flight", "Flight stopped!");
}