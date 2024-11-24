#include "MainWidget.h"
#include "TabWidget.h"
#include "GPSWidget.h"
#include "ROSThread.h"
#include "VideoStreamWidget.h"
#include "videoSubscriber.h"
#include <rclcpp/rclcpp.hpp>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include <QTabWidget>

TabWidget::TabWidget(QWidget *parent) : QWidget(parent)
{
    QTabWidget *tabWidget = new QTabWidget;

    // Create main tab
    QWidget *tab1 = new QWidget;
    QVBoxLayout *mainLayout = new QVBoxLayout;
    MainWidget *mainWidget = new MainWidget(this);
    mainLayout->addWidget(mainWidget);
    tab1->setLayout(mainLayout);

    // Create video tab
    QWidget *tab2 = new QWidget;
    QVBoxLayout *layout2 = new QVBoxLayout;
    VideoStreamWidget *videoWidget = new VideoStreamWidget(this);
    layout2->addWidget(videoWidget);
    tab2->setLayout(layout2);

    // Add tabs to the QTabWidget
    tabWidget->addTab(tab1, "Main");
    tabWidget->addTab(tab2, "Video");

    // Start nodes running in separate threads
    auto gps_publisher_node = mainWidget->gpsWidget->getPublisher();
    auto gps_subscriber_node = mainWidget->gpsWidget->getSubscriber();
    auto video_subscriber_node = videoWidget->getVideoSubscriber();

    gps_publisher_thread = new ROSThread(gps_publisher_node);
    gps_subscriber_thread = new ROSThread(gps_subscriber_node);
    video_subscriber_thread = new ROSThread(video_subscriber_node);

    gps_publisher_thread->start();
    std::cout << "GPS publisher thread started" << std::endl;

    gps_subscriber_thread->start();
    std::cout << "GPS subscriber thread started" << std::endl;

    video_subscriber_thread->start();
    std::cout << "Video subscriber thread started" << std::endl;

    // Set up main window
    QVBoxLayout *tabLayout = new QVBoxLayout(this);
    tabLayout->addWidget(tabWidget);
    setLayout(tabLayout);    
    tabWidget->resize(400, 300);
}

TabWidget::~TabWidget()
{
    if (gps_publisher_thread)
    {
        gps_publisher_thread->quit();
        gps_publisher_thread->wait();
        delete gps_publisher_thread;
        std::cout << "GPS publisher thread closed" << std::endl;
    }

    if (gps_subscriber_thread)
    {
        gps_subscriber_thread->quit();
        gps_subscriber_thread->wait();
        delete gps_subscriber_thread;
        std::cout << "GPS subscriber thread closed" << std::endl;
    }

    if (video_subscriber_thread)
    {
        video_subscriber_thread->quit();
        video_subscriber_thread->wait();
        delete video_subscriber_thread;
        std::cout << "Video subscriber thread closed" << std::endl;
    }
}
