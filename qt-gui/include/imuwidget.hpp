#pragma once

#include <QWidget>
#include <QTextBrowser>
#include "ROSThread.h"
#include "imusubscriber.hpp"

class IMUWidget : public QWidget {
    Q_OBJECT

public:
    explicit IMUWidget(QTextBrowser *data = nullptr);
    ~IMUWidget() override;

public slots:
    void updateText(
        float64 orientation_covar[9],
        float64 angular_vel_covar[9],
        float64 linear_accel_covar[9]);

private:
    QTextBrowser *imu_textbox_;
    ROSThread *imu_thread;
    std::shared_ptr<IMUSubscriber> imuSubscriber_;
    std::unique_ptr<ROSThread> rosThread_;
};
