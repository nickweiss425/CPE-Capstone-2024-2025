#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include "ROSThread.h"
#include "connectionstatussubscriber.hpp"

class ConnectionStatusWidget : public QWidget {
    Q_OBJECT

public:
    explicit ConnectionStatusWidget(QLabel *label = nullptr,
        QPushButton *startFlightbutton = nullptr);
    ~ConnectionStatusWidget() override;

public slots:
    void updateLabel(int status);

private:
    QLabel *connection_status_label_;
    QPushButton *start_flight_button_;
    ROSThread *connection_status_thread_;
    std::shared_ptr<ConnectionStatusSubscriber> connection_status_subscriber_;
    std::unique_ptr<ROSThread> rosThread_;
};

