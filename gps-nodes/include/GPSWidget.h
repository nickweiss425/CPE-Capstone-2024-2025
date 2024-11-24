#ifndef GPSWIDGET_H
#define GPSWIDGET_H

#include "subscriber.h"
#include "publisher.h"
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <QString>

class MinimalPublisher;
class MinimalSubscriber;

class GPSWidget : public QWidget
{
    Q_OBJECT

public:
    explicit GPSWidget(QWidget *parent = nullptr);

    // Accessors for GPS publisher and subscriber
    std::shared_ptr<MinimalPublisher> getPublisher() const { return publisher_; }
    std::shared_ptr<MinimalSubscriber> getSubscriber() const { return subscriber_; }

public slots:
    // Methods to update GPS coordinates on the display
    void updateTalkerCoordinates(double latitude, double longitude, double altitude);
    void updateListenerCoordinates(double latitude, double longitude, double altitude);

private:
    // UI Components
    QLabel *talkerLabel;
    QLabel *listenerLabel;

    // ROS Communication
    std::shared_ptr<MinimalPublisher> publisher_;
    std::shared_ptr<MinimalSubscriber> subscriber_;
};

#endif // GPSWIDGET_H
