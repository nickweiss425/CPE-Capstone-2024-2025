#include "gpswidget.hpp"
#include "datalogger.hpp"

using std::placeholders::_1;

/**
 * @brief Constructs a GPSWidget object.
 * 
 * This constructor initializes a GPSWidget object with the provided QLineEdit pointers for longitude and latitude.
 * It creates a GPSSubscriber object and connects its coordinatesUpdated signal to the updateLocation slot of the GPSWidget.
 * It also creates a ROSThread object and starts it with the GPSSubscriber.
 * 
 * @param longitude A pointer to the QLineEdit object representing the longitude.
 * @param latitude A pointer to the QLineEdit object representing the latitude.
 */
GPSWidget::GPSWidget(QTextBrowser *sensorDataTextBrowser, QLineEdit *longitude, QLineEdit *latitude)
    : sensorData_textBrowser_(sensorDataTextBrowser), longitudeBox_(longitude), latitudeBox_(latitude), gpsSubscriber_(nullptr) {
    gpsSubscriber_ = std::make_shared<GPSSubscriber>();

    if (!gpsSubscriber_) {
        std::cerr << "Error: Failed to create GPSSubscriber!" << std::endl;
        return;
    }

    if (!QObject::connect(gpsSubscriber_.get(), &GPSSubscriber::coordinatesUpdated, this, &GPSWidget::updateLocation)) {
        std::cerr << "Error: Failed to connect GPSSubscriber signal!" << std::endl;
    }

    // Convert the GPSSubscriber to a ROS thread and start it
    rosThread_ = std::make_unique<ROSThread>(gpsSubscriber_);
    rosThread_->start();
}

/**
 * @brief Destructor for the GPSWidget class.
 * 
 * This destructor is responsible for cleaning up any resources used by the GPSWidget object.
 * It checks if the ROS thread is running and if so, it quits the thread and waits for it to finish.
 * Finally, it shuts down the ROS node.
 */
GPSWidget::~GPSWidget() {
    if (rosThread_ && rosThread_->isRunning()) {
        rosThread_->quit();
        rosThread_->wait();
    }
    rclcpp::shutdown();
}

/**
 * @brief Slot to update the location information.
 * 
 * This slot is called when the GPSSubscriber emits the coordinatesUpdated signal.
 * It updates the longitude and latitude text boxes with the new values.
 * It also emits the coordinatesUpdated signal with the new longitude and latitude values.
 * 
 * @param longitude The new longitude value.
 * @param latitude The new latitude value.
 */
void GPSWidget::updateLocation(const double &longitude, const double &latitude, const double &altitude) {

    // Create nicely formatted strings for the longitude and latitude
    QString longitudeText = QString("%1%2")
        .arg(qAbs(longitude))
        .arg(longitude >= 0 ? "° E" : "° W");

    QString latitudeText = QString("%1%2")
        .arg(qAbs(latitude))
        .arg(latitude >= 0 ? "° N" : "° S");

    longitudeBox_->setText(longitudeText);
    latitudeBox_->setText(latitudeText);

    // Receive the altitude along with the coordinate if needed for data logging
    QString altitudeText = QString("Altitude: %1 m\n").arg(altitude);

    // Send the updated coordinates to the main window
    emit coordinatesUpdated(longitude, latitude);

    sensorData_textBrowser_->append(longitudeText);
    sensorData_textBrowser_->append(latitudeText);
    sensorData_textBrowser_->append(altitudeText);

    // Log the data if recording is enabled
    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        dataLogger_->log_data(longitudeText.toStdString());
        dataLogger_->log_data(latitudeText.toStdString());
        dataLogger_->log_data(altitudeText.toStdString());
    }
}
