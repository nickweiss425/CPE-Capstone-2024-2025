#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QTextBrowser>
#include "ROSThread.h"
#include "gpssubscriber.hpp"

class GPSWidget : public QWidget {
    Q_OBJECT

public:
    explicit GPSWidget(QTextBrowser *sensorDataTextBrowser = nullptr, QLineEdit *longitude = nullptr, QLineEdit *latitude = nullptr);
    ~GPSWidget() override;

signals:
    void coordinatesUpdated(const double &longitude, const double &latitude);

public slots:
    void updateLocation(const double &longitude, const double &latitude, const double &altitude);

private:
    QTextBrowser *sensorData_textBrowser_;
    QLineEdit *longitudeBox_;
    QLineEdit *latitudeBox_;
    ROSThread *gps_thread;
    std::shared_ptr<GPSSubscriber> gpsSubscriber_;
    std::unique_ptr<ROSThread> rosThread_;
};
