#include "imuwidget.hpp"

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

void IMUWidget::updateText(float64 orientation_covar[9], float64 angular_vel_covar[9], float64 linear_accel_covar[9]) {
    imu_textbox_->append(
        QString::number(orientation_covar[0]),
        QString::number(orientation_covar[1]),
        QString::number(orientation_covar[2]));
}
