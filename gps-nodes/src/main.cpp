#include "main.h"
#include "MainWidget.h"
#include <rclcpp/rclcpp.hpp>
#include <QApplication>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWidget mainWidget;
    mainWidget.show();

    int result = app.exec();
    rclcpp::shutdown();

    return result;
}

