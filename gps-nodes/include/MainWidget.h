#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <memory>
#include "ROSThread.h"
#include <rclcpp/rclcpp.hpp>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include "GPSWidget.h"

class GPSWidget;

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

private:
    GPSWidget *gpsWidget;
    ROSThread *publisher_thread;
    ROSThread *subscriber_thread;
};

#endif
