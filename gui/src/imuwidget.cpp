#include "imuwidget.hpp"
#include "datalogger.hpp"
#include <array>
#include <iostream>
#include <QMetaType>

using std::placeholders::_1;

/**
 * @brief Constructs an IMUWidget object.
 * 
 * This constructor initializes an IMUWidget object with the given QTextBrowser pointer.
 * It creates an IMUSubscriber, registers signal types, connects signals, and starts a ROS thread.
 * 
 * @param data A pointer to the QTextBrowser object used to display IMU data.
 */
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

/**
 * @brief Destroys an IMUWidget object.
 * 
 * This destructor stops the ROS thread and shuts down the ROS node.
 */
IMUWidget::~IMUWidget() {
    if (rosThread_ && rosThread_->isRunning()) {
        rosThread_->quit();
        rosThread_->wait();
    }
    rclcpp::shutdown();
}

/**
 * @brief Updates the IMU data displayed in the QTextBrowser.
 * 
 * This function updates the IMU data displayed in the QTextBrowser with the given orientation, angular velocity, and linear acceleration.
 * It also logs the data if the DataLogger is recording.
 * 
 * @param orientation The orientation data to display.
 * @param angular_velocity The angular velocity data to display.
 * @param linear_acceleration The linear acceleration data to display.
 */
void IMUWidget::updateText(
    const geometry_msgs::msg::Quaternion orientation,
    const geometry_msgs::msg::Vector3 angular_velocity,
    const geometry_msgs::msg::Vector3 linear_acceleration)
{
    // Print quaternion data (x, y, z, w)
    QString quaternionText = QString("Orientation: x=%1, y=%2, z=%3, w=%4")
                         .arg(orientation.x)
                         .arg(orientation.y)
                         .arg(orientation.z)
                         .arg(orientation.w);
    imu_textbox_->append(quaternionText);

    // Print angular velocity data (x, y, z)
    QString velocityText = QString("Angular Velocity: x=%1, y=%2, z=%3")
                         .arg(angular_velocity.x)
                         .arg(angular_velocity.y)
                         .arg(angular_velocity.z);
    imu_textbox_->append(velocityText);

    // Print linear acceleration data (x, y, z)
    QString accelerationText = QString("Linear Acceleration: x=%1, y=%2, z=%3\n")
                         .arg(linear_acceleration.x)
                         .arg(linear_acceleration.y)
                         .arg(linear_acceleration.z);
    imu_textbox_->append(accelerationText);

    auto dataLogger_ = DataLogger::getInstance();
    if (dataLogger_->getRecording()) {
        dataLogger_->log_data(quaternionText.toStdString());
        dataLogger_->log_data(velocityText.toStdString());
        dataLogger_->log_data(accelerationText.toStdString());
    }
}

