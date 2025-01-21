#pragma once

#include <QWidget>
#include <QLineEdit>
#include "ROSThread.h"
#include "gpssubscriber.hpp"

class GPSWidget : public QWidget {
    Q_OBJECT

public:
    explicit GPSWidget(QLineEdit *longitude = nullptr, QLineEdit *latitude = nullptr, QLineEdit *altitude = nullptr);
    ~GPSWidget() override;

public slots:
    void updateLocation(const double &longitude, const double &latitude, const double &altitude);

private:
    QLineEdit *longitudeBox_;
    QLineEdit *latitudeBox_;
    QLineEdit *altitudeBox_;
    ROSThread *gps_thread;
    std::shared_ptr<GPSSubscriber> gpsSubscriber_;
    std::unique_ptr<ROSThread> rosThread_;
};
