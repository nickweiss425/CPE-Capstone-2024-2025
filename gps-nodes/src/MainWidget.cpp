#include "MainWidget.h"
#include "GPSWidget.h"
#include "ROSThread.h"
#include <rclcpp/rclcpp.hpp>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>

MainWidget::MainWidget(QWidget *parent) : QWidget(parent)
{
    setWindowTitle("ROS 2 Qt Application");
    resize(800, 600);

    gpsWidget = new GPSWidget(this);
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(gpsWidget);
    setLayout(layout);

    auto publisher_node = gpsWidget->getPublisher();
    auto subscriber_node = gpsWidget->getSubscriber();

    publisher_thread = new ROSThread(publisher_node);
    subscriber_thread = new ROSThread(subscriber_node);

    publisher_thread->start();
    subscriber_thread->start();
}

MainWidget::~MainWidget()
{
    if (publisher_thread)
    {
        publisher_thread->quit();
        publisher_thread->wait();
        delete publisher_thread;
    }
    
    if (subscriber_thread)
    {
        subscriber_thread->quit();
        subscriber_thread->wait();
        delete subscriber_thread;
    }
}
