#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QQuickWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), videoScene(new QGraphicsScene(this)) {
    ui->setupUi(this);

    // Set up the map scene in the mapView
    ui->mapView->setSource(QUrl::fromLocalFile("../map.qml"));
    ui->mapView->setResizeMode(QQuickWidget::SizeRootObjectToView);

    // Set up the video scene in the videoView
    ui->videoView->setScene(videoScene);
    videoScene->addText("Video Placeholder");

    // Set default placeholder text for coordinate boxes
    ui->latitudeBox->setText("0.000000");
    ui->longitudeBox->setText("0.000000");

    // Connect buttons to their slots
    setupConnections();
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::setupConnections() {
    connect(ui->startRecordingButton, &QPushButton::clicked, this, &MainWindow::startRecording);
    connect(ui->stopRecordingButton, &QPushButton::clicked, this, &MainWindow::stopRecording);
    connect(ui->stopFlightButton, &QPushButton::clicked, this, &MainWindow::stopFlight);
}

void MainWindow::addWaypoint() {
    
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

void MainWindow::updateCoordinates() {
    ui->latitudeBox->setText("37.7749");
    ui->longitudeBox->setText("-122.4194");
}