#ifndef TABWIDGET_H
#define TABWIDGET_H

#include <QWidget>
#include <memory>
#include "ROSThread.h"
#include <rclcpp/rclcpp.hpp>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include "GPSWidget.h"
#include "MainWidget.h"
#include "VideoStreamWidget.h"
#include <QTabWidget>

class VideoStreamWidget;
class ROSThread;
class MainWidget;
class VideoSubscriber;

class TabWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TabWidget(QWidget *parent = nullptr);
    ~TabWidget();

private:
    QTabWidget *tabWidget;
    QWidget *mainTab;
    QWidget *videoTab;
    MainWidget *mainWidget;
    VideoStreamWidget *videoStreamWidget;

    std::shared_ptr<VideoSubscriber> videoSubscriber;

    ROSThread *gps_publisher_thread;
    ROSThread *gps_subscriber_thread;
    ROSThread *video_subscriber_thread;
};

#endif // TABWIDGET_H
