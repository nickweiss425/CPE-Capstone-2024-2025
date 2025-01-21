#include "videowidget.hpp"

VideoStreamWidget::VideoStreamWidget(QWidget *parent)
    : QWidget(parent), videoSubscriber_(nullptr)
{
    rclcpp::init(0, nullptr);
    videoSubscriber_ = std::make_shared<VideoSubscriber>();

    if (!videoSubscriber_) {
        std::cerr << "Error: Failed to create VideoSubscriber!" << std::endl;
        return;
    }

    imageLabel_ = new QLabel(this);
    imageLabel_->setAlignment(Qt::AlignCenter);
    imageLabel_->setStyleSheet("background-color: black;");

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(imageLabel_);
    setLayout(layout);

    if (!connect(videoSubscriber_.get(), &VideoSubscriber::imageReceived, this, &VideoStreamWidget::updateImage)) {
        std::cerr << "Error: Failed to connect VideoSubscriber signal!" << std::endl;
    }

    rosThread_ = std::make_unique<ROSThread>(videoSubscriber_);
    rosThread_->start();
}

VideoStreamWidget::~VideoStreamWidget() {
    if (rosThread_ && rosThread_->isRunning()) {
        rosThread_->quit();
        rosThread_->wait();
    }
    rclcpp::shutdown();
}

void VideoStreamWidget::updateImage(const QImage &image)
{
	if (image.isNull()) {
        std::cerr << "Warning: Received null image!" << std::endl;
        return;
    }
	imageLabel_->setPixmap(QPixmap::fromImage(image));
}