#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include "flightstates.hpp"

#include <QMessageBox>
#include <QQuickWidget>
#include <QMediaPlayer>
#include <QVideoWidget>

/**
 * @brief Constructor for the MainWindow class.
 * 
 * This constructor initializes the MainWindow object and sets up the user interface.
 * It also sets up the GPS coordinates display, IMU data stream, flight state publisher/subscriber,
 * and the video widget. Additionally, it creates an exit handler and connects buttons to their slots.
 * 
 * @param parent The parent widget.
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // Set up the GPS coordinates display
    gpsWidget = new GPSWidget(ui->longitudeBox, ui->latitudeBox);

    // Set up the IMU data stream
    imuWidget = new IMUWidget(ui->sensorData);

    // Set up the Connection Status widget
    connectionStatusWidget = new ConnectionStatusWidget(ui->connectionStatusLabel, ui->startFlightButton);

    // Set up the flight state publisher/subscriber
    statePublisher = new StatePublisher();
    stateSubscriber = new StateSubscriber();

    // Set up the video widget in the videoView
    ui->videoWidget->setObjectName("videoWidget");

    ui->sensorData->setText("");

    // Create exit handler
    std::signal(SIGINT, MainWindow::signalHandler);

    // Connect buttons to their slots
    setupConnections();
}

/**
 * @brief Destructor for the MainWindow class.
 * 
 * This destructor stops the flight, shuts down the data logger, and deletes the user interface.
 */
MainWindow::~MainWindow() {
    stopFlight();
    DataLogger::shutdown();
    delete ui;
    rclcpp::shutdown();
}

/**
 * @brief Sets up the connections between the buttons and their respective slots.
 */
void MainWindow::setupConnections() {
    // Connect flight state and recording buttons to their respective slots
    connect(ui->startFlightButton, &QPushButton::clicked, this, &MainWindow::startFlight);
    connect(ui->toggleRecordingButton, &QPushButton::clicked, this, &MainWindow::toggleRecording);
    connect(ui->stopFlightButton, &QPushButton::clicked, this, &MainWindow::stopFlight);

    // Connect the waypoint control buttons to their respective slots
    connect(ui->confirmHoverButton, &QPushButton::clicked, this, &MainWindow::handleWaypointUpdate);
    connect(ui->waypointManager, &WaypointManager::getWaypointAttributes, this, &MainWindow::updateWaypointAttributes);
    connect(this, &MainWindow::setWaypointAttributes, ui->waypointManager, &WaypointManager::updateWaypointAttributes);
    connect(stateSubscriber, &StateSubscriber::stateReceived, ui->waypointManager, &WaypointManager::handleDroneStateReceive);
    connect(ui->waypointManager, &WaypointManager::resetWaypointButtons, this, &MainWindow::resetWaypointAttributes);

    // Wait for the map to load before connecting signals
    QObject *map = ui->mapView->rootObject();
    if (map) {
        // Connect the waypoint manager backend to the map
        connect(map, SIGNAL(waypointAdded(double, double)), ui->waypointManager, SLOT(onWaypointAdded(double, double)));
        connect(map, SIGNAL(waypointSelected(int)), ui->waypointManager, SLOT(onWaypointSelected(int)));
        connect(map, SIGNAL(waypointRemoved(int)), ui->waypointManager, SLOT(onWaypointRemoved(int)));
        connect(gpsWidget, &GPSWidget::coordinatesUpdated, ui->waypointManager, &WaypointManager::getDronePosition);
        connect(ui->waypointManager, &WaypointManager::updateDronePosition, this, [this](double lat, double lon) {
            QMetaObject::invokeMethod(ui->mapView->rootObject(), "updateDronePosition", Q_ARG(QVariant, lat), Q_ARG(QVariant, lon));
        });
        connect(ui->waypointManager, &WaypointManager::updateWaypointVisual, this, [this](int index, int type) {
            QMetaObject::invokeMethod(ui->mapView->rootObject(), "setWaypointData", Q_ARG(QVariant, index), Q_ARG(QVariant, type));
        });
    }

    initializeButtons();
}

/**
 * @brief Initializes the buttons in the main window.
 * 
 * This function sets the buttons to their initial states, enabling or disabling them as necessary.
 * The start flight button is enabled, while the toggle recording button and stop flight button are disabled.
 * The hover control buttons and confirm hover button are enabled.
 */
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

/**
 * @brief Unlocks the buttons after the flight has started.
 * 
 * This function disables the start flight button and enables the toggle recording and stop flight buttons.
 */
void MainWindow::unlockButtons() {
    // Enable buttons after start flight
    ui->startFlightButton->setEnabled(false);
    ui->toggleRecordingButton->setEnabled(true);
    ui->stopFlightButton->setEnabled(true);
}

/**
 * @brief Starts the flight.
 * 
 * This function publishes the takeoff state to the flight state publisher.
 */
void MainWindow::startFlight() {
    statePublisher->publish_state(flight_states::FlightState::TAKEOFF);
    unlockButtons();
}

/**
 * @brief Toggles the recording of data.
 * 
 * This function toggles the recording of data by the data logger.
 * If the data logger is recording, it stops the recording and closes the file.
 * If the data logger is not recording, it starts the recording and creates a new file.
 */
void MainWindow::toggleRecording() {
    auto dataLogger_ = DataLogger::getInstance();
    dataLogger_->switchRecording();

    if (!dataLogger_->getRecording()) {
        dataLogger_->close_file();
        ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
    } else {
        dataLogger_->create_file();
        ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\271", nullptr));
    }
}

/**
 * @brief Stops the flight.
 * 
 * This function publishes the landed state to the flight state publisher.
 * It also changes the text of the toggle recording button to the record icon.
 */
void MainWindow::stopFlight() {
    statePublisher->publish_state(flight_states::FlightState::LANDED);
    ui->toggleRecordingButton->setText(QCoreApplication::translate("MainWindow", "\342\217\272", nullptr));
    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        dataLogger_->switchRecording();
    }
    ui->startFlightButton->setEnabled(true);
    ui->stopFlightButton->setEnabled(false);
}

/**
 * @brief Updates the GPS coordinates display.
 * 
 * This function updates the GPS coordinates display with the given latitude and longitude.
 * 
 * @param lat The latitude to display.
 * @param lon The longitude to display.
 */
void MainWindow::updateWaypointAttributes(double radius, double altitude, double duration, int type) {
    // First reset all buttons to initial state
    ui->hoverGroup->setExclusive(false);
    ui->hoverControlButton1->setChecked(false);
    ui->hoverControlButton2->setChecked(false);
    ui->hoverControlButton3->setChecked(false);
    ui->hoverControlButton4->setChecked(false);
    ui->hoverGroup->setExclusive(true);

    // Clear text fields first
    ui->radiusBox->clear();
    ui->altitudeBox->clear();
    ui->durationBox->clear();

    // If no waypoint is selected (type == 5 or -1)
    if (type == 5 || type == -1) {
        /*ui->hoverControlButton1->setCheckable(false);
        ui->hoverControlButton2->setCheckable(false);
        ui->hoverControlButton3->setCheckable(false);
        ui->hoverControlButton4->setCheckable(false);
        ui->confirmHoverButton->setCheckable(false);*/
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

    // Update the button state so they display correctly
    ui->hoverControlButton1->update();
    ui->hoverControlButton2->update();
    ui->hoverControlButton3->update();
    ui->hoverControlButton4->update();
    
}

/**
 * @brief Handles the update of the waypoint attributes.
 * 
 * This function retrieves the selected waypoint type from the UI and emits a signal to set the waypoint attributes.
 * It also resets the waypoint attributes and updates the UI accordingly.
 * 
 * @return void
 */
void MainWindow::handleWaypointUpdate() {
    int waypointType = ui->hoverGroup->checkedId();
    emit setWaypointAttributes(ui->radiusBox->text().toDouble(), ui->altitudeBox->text().toDouble(), ui->durationBox->text().toDouble(), waypointType);
    
    resetWaypointAttributes();

    /*switch (waypointType) {
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
    }*/
}

/**
 * @brief Resets the attributes of the waypoint controls in the main window.
 * 
 * This function sets the checkable property of the hover control buttons to false,
 * and sets the checked property of the hover control buttons to false. It also updates
 * the hover control buttons to reflect the changes. Finally, it sets the checked property
 * of the confirm hover button to false.
 */
void MainWindow::resetWaypointAttributes() {
    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        dataLogger_->log_data("resetWaypointAttributes");
    }

    ui->radiusBox->clear();
    ui->altitudeBox->clear();
    ui->durationBox->clear();

    ui->hoverGroup->setExclusive(false);
    ui->hoverControlButton1->setChecked(false);
    ui->hoverControlButton2->setChecked(false);
    ui->hoverControlButton3->setChecked(false);
    ui->hoverControlButton4->setChecked(false);
    ui->hoverGroup->setExclusive(true);

    ui->hoverControlButton1->setCheckable(false);
    ui->hoverControlButton2->setCheckable(false);
    ui->hoverControlButton3->setCheckable(false);
    ui->hoverControlButton4->setCheckable(false);

    ui->confirmHoverButton->setChecked(false);

    ui->hoverControlButton1->update();
    ui->hoverControlButton2->update();
    ui->hoverControlButton3->update();
    ui->hoverControlButton4->update();
}

/**
 * @brief Signal handler for the main window.
 * 
 * This function shuts down the ROS2 node and exits the program when the SIGINT signal is received.
 * 
 * @param sig The signal number.
 */
void MainWindow::signalHandler(int) {
    rclcpp::shutdown();
    exit(0);
}
