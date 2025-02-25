#include "videowidget.hpp"

/**
 * @brief Constructs a VideoStreamWidget object.
 * 
 * This constructor initializes a VideoStreamWidget object with the given parent widget.
 * It also initializes the videoSubscriber_ member variable and sets up the user interface.
 * 
 * @param parent The parent widget.
 */
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

/**
 * @brief Destructor for the VideoStreamWidget class.
 * 
 * This destructor is responsible for cleaning up any resources used by the VideoStreamWidget object.
 * It checks if the ROS thread is running and if so, it stops the thread by calling `quit()` and `wait()`.
 * Finally, it shuts down the ROS node by calling `rclcpp::shutdown()`.
 */
VideoStreamWidget::~VideoStreamWidget() {
    if (rosThread_ && rosThread_->isRunning()) {
        rosThread_->quit();
        rosThread_->wait();
    }
    rclcpp::shutdown();
}

/**
 * @brief Updates the image displayed in the video stream widget.
 *
 * This function sets the pixmap of the imageLabel_ to the given image.
 * If the image is null, a warning message is printed to the standard error stream.
 *
 * @param image The image to be displayed.
 */
void VideoStreamWidget::updateImage(const QImage &image)
{
	if (image.isNull()) {
        std::cerr << "Warning: Received null image!" << std::endl;
        return;
    }
	imageLabel_->setPixmap(QPixmap::fromImage(image));
}