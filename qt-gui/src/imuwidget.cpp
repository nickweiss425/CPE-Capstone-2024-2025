#include "imuwidget.hpp"
#include <array>
#include <iostream>
#include <QMetaType>

using std::placeholders::_1;


IMUWidget::IMUWidget(QTextBrowser *data)
    : imu_textbox_(data), imuSubscriber_(nullptr) {
    /* create subscriber*/
    imuSubscriber_ = std::make_shared<IMUSubscriber>();
    if (!imuSubscriber_) {
        std::cerr << "Error: Failed to create IMUSubscriber!" << std::endl;
        return;
    }

    /* register quaternion as a signal type */
    qRegisterMetaType<geometry_msgs::msg::Quaternion>("geometry_msgs::msg::Quaternion");
    /* register vector3 as a signal type */
    qRegisterMetaType<geometry_msgs::msg::Vector3>("geometry_msgs::msg::Vector3");
    
    /* connect qsignal */
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
    const geometry_msgs::msg::Quaternion orientation,
    const geometry_msgs::msg::Vector3 angular_velocity,
    const geometry_msgs::msg::Vector3 linear_acceleration)
{
    // Print quaternion data (x, y, z, w)
    imu_textbox_->append(QString("Orientation: x=%1, y=%2, z=%3, w=%4")
                         .arg(orientation.x)
                         .arg(orientation.y)
                         .arg(orientation.z)
                         .arg(orientation.w));

    // Print angular velocity data (x, y, z)
    imu_textbox_->append(QString("Angular Velocity: x=%1, y=%2, z=%3")
                         .arg(angular_velocity.x)
                         .arg(angular_velocity.y)
                         .arg(angular_velocity.z));

    // Print linear acceleration data (x, y, z)
    imu_textbox_->append(QString("Linear Acceleration: x=%1, y=%2, z=%3\n")
                         .arg(linear_acceleration.x)
                         .arg(linear_acceleration.y)
                         .arg(linear_acceleration.z));
}

