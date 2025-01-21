#pragma once
#include <QWidget>
#include <QImage>
#include <QLabel>
#include <QVBoxLayout>
#include "ROSThread.h"
#include "videosubscriber.hpp"

class VideoStreamWidget : public QWidget {
    Q_OBJECT

public:
    explicit VideoStreamWidget(QWidget *parent = nullptr);
    std::shared_ptr<VideoSubscriber> getVideoSubscriber() const {
	    return videoSubscriber_;
	   }
    ~VideoStreamWidget() override;

public slots:
    void updateImage(const QImage &image);

private:
    QLabel *imageLabel_;
    std::shared_ptr<VideoSubscriber> videoSubscriber_;
    ROSThread *video_subscriber_thread;
    std::unique_ptr<ROSThread> rosThread_;
};
