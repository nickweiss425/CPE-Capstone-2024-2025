#include "gpswidget.hpp"
#include "dataLogger.hpp"

using std::placeholders::_1;

GPSWidget::GPSWidget(QLineEdit *longitude, QLineEdit *latitude, QLineEdit *altitude)
    : longitudeBox_(longitude), latitudeBox_(latitude), altitudeBox_(altitude), gpsSubscriber_(nullptr) {
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
    longitudeBox_->setText(QString::number(longitude));
    latitudeBox_->setText(QString::number(latitude));
    altitudeBox_->setText(QString::number(altitude));
    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        dataLogger_->log_data(std::string("hello world"));
    }
}