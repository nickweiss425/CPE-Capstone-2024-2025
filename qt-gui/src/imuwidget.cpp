#include "imuwidget.hpp"
#include <iostream>

using std::placeholders::_1;

IMUWidget::IMUWidget(QTextBrowser *data)
    : imu_textbox_(data), imuSubscriber_(nullptr) {
    imuSubscriber_ = std::make_shared<IMUSubscriber>();

    if (!imuSubscriber_) {
        std::cerr << "Error: Failed to create IMUSubscriber!" << std::endl;
        return;
    }

    if (!QObject::connect(imuSubscriber_.get(), &IMUSubscriber::dataUpdated, this, &IMUWidget::updateText)) {
        std::cerr << "Error: Failed to connect IMUSubscriber signal!" << std::endl;
    }

    rosThread_ = std::make_unique<ROSThread>(imuSubscriber_);
    rosThread_->start();
}

IMUWidget::~IMUWidget() {
    if (rosThread_ && rosThread_->isRunning()) {
        rosThread_->quit();
        rosThread_->wait();
    }
    rclcpp::shutdown();
}

void IMUWidget::updateText(
    const std::array<double, 9> orientation_covariance,
    const std::array<double, 9> angular_velocity_covariance,
    const std::array<double, 9> linear_acceleration_covariance)
{
    imu_textbox_->append("TESTING DATA ");
}
