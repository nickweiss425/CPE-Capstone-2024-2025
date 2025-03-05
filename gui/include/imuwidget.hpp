#pragma once

#include <QWidget>
#include <QTextBrowser>
#include "ROSThread.h"
#include "imusubscriber.hpp"

class IMUWidget : public QWidget {
    Q_OBJECT

public:
    explicit IMUWidget(QTextBrowser *sensorDataTextBrowser = nullptr);
    ~IMUWidget() override;

public slots:
    void updateText(
        const geometry_msgs::msg::Quaternion orientation,
        const geometry_msgs::msg::Vector3 angular_velocity,
        const geometry_msgs::msg::Vector3 linear_acceleration
    );

private:
    QTextBrowser *sensorData_textBrowser_;
    ROSThread *imu_thread;
    std::shared_ptr<IMUSubscriber> imuSubscriber_;
    std::unique_ptr<ROSThread> rosThread_;
};
