#include "gpswidget.hpp"
#include "datalogger.hpp"

using std::placeholders::_1;

GPSWidget::GPSWidget(QLineEdit *longitude, QLineEdit *latitude)
    : longitudeBox_(longitude), latitudeBox_(latitude), gpsSubscriber_(nullptr) {
    gpsSubscriber_ = std::make_shared<GPSSubscriber>();

    if (!gpsSubscriber_) {
        std::cerr << "Error: Failed to create GPSSubscriber!" << std::endl;
        return;
    }

    if (!QObject::connect(gpsSubscriber_.get(), &GPSSubscriber::coordinatesUpdated, this, &GPSWidget::updateLocation)) {
        std::cerr << "Error: Failed to connect GPSSubscriber signal!" << std::endl;
    }

    rosThread_ = std::make_unique<ROSThread>(gpsSubscriber_);
    rosThread_->start();
}

GPSWidget::~GPSWidget() {
    if (rosThread_ && rosThread_->isRunning()) {
        rosThread_->quit();
        rosThread_->wait();
    }
    rclcpp::shutdown();
}

void GPSWidget::updateLocation(const double &longitude, const double &latitude, const double &altitude) {

    QString longitudeText = QString("%1%2\n")
        .arg(qAbs(longitude))
        .arg(longitude >= 0 ? "° E" : "° W");

    QString latitudeText = QString("%1%2\n")
        .arg(qAbs(latitude))
        .arg(latitude >= 0 ? "° N" : "° S");

    longitudeBox_->setText(longitudeText);
    latitudeBox_->setText(latitudeText);

    QString altitudeText = QString("Altitude: %1\n").arg(altitude);

    emit coordinatesUpdated(longitude, latitude);

    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        dataLogger_->log_data(longitudeText.toStdString());
        dataLogger_->log_data(latitudeText.toStdString());
        dataLogger_->log_data(altitudeText.toStdString());
    }
}
