#include "MainWidget.h"
#include "GPSWidget.h"
#include <rclcpp/rclcpp.hpp>
#include <QVBoxLayout>
#include <QWidget>

MainWidget::MainWidget(QWidget *parent) : QWidget(parent)
{
    gpsWidget = new GPSWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(gpsWidget);
    setLayout(mainLayout);
}
