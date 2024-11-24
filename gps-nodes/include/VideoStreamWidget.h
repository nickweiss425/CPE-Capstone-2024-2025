#ifndef VIDEOSTREAMWIDGET_H
#define VIDEOSTREAMWIDGET_H

#include <QWidget>
#include <QImage>
#include <QLabel>
#include <QVBoxLayout>
#include "videoSubscriber.h"

class VideoStreamWidget : public QWidget {
    Q_OBJECT

public:
    explicit VideoStreamWidget(QWidget *parent = nullptr);
    std::shared_ptr<VideoSubscriber> getVideoSubscriber() const {
	    return videoSubscriber_;
	   }

public slots:
    void updateImage(const QImage &image);

private:
    QLabel *imageLabel_;
    std::shared_ptr<VideoSubscriber> videoSubscriber_;
    //ROSThread *video_subscriber_thread;
};

#endif // VIDEOSTREAMWIDGET_H
