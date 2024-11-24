#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include "GPSWidget.h"
#include "VideoStreamWidget.h"
#include <QTabWidget>

class GPSWidget;

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MainWidget(QWidget *parent = nullptr);
    GPSWidget *gpsWidget;
};

#endif // MAINWIDGET_H
