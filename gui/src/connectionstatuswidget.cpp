#include "connectionstatuswidget.hpp"
#include <array>
#include <iostream>

using std::placeholders::_1;

ConnectionStatusWidget::ConnectionStatusWidget(QLabel *label)
    :connection_status_label_(label), connection_status_subscriber_(nullptr)
{
    /* create subscriber */
    connection_status_subscriber_ = std::make_shared<ConnectionStatusSubscriber>();
    if (!connection_status_subscriber_) {
        std::cerr << "Error: Failed to create connection status subscriber!" << std::endl;
        return;
    }
    
    /* connect qsignal */
    if (!QObject::connect(connection_status_subscriber_.get(),
        &ConnectionStatusSubscriber::connectionStatusUpdate, this,
        &ConnectionStatusWidget::updateLabel))
    {
        std::cerr << "Error: Failed to connect connection status signal!" << std::endl;
    }

    rosThread_ = std::make_unique<ROSThread>(connection_status_subscriber_);
    rosThread_->start();
}


ConnectionStatusWidget::~ConnectionStatusWidget() {
    if (rosThread_ && rosThread_->isRunning()) {
        rosThread_->quit();
        rosThread_->wait();
    }
    rclcpp::shutdown();
}


/* connection status label update */
void ConnectionStatusWidget::updateLabel(int status)
{
    if (!status)
    {
        connection_status_label_->setText("DISCONNECTED");
    }
    else
    {
        connection_status_label_->setText("CONNECTED");
    }
}

