#include "main.h"
#include "TabWidget.h"
#include "MainWidget.h"
#include "VideoStreamWidget.h"
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QTabWidget>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>


int main(int argc, char *argv[])
{
    std::cout << "Initializing ROS..." << std::endl;
    rclcpp::init(argc, argv);

    std::cout << "Starting Qt application..." << std::endl;
    QApplication app(argc, argv);

    std::cout << "Creating TabWidget..." << std::endl;
    TabWidget tabWidget;
    tabWidget.setWindowTitle("Northrop Grumman Cal Poly Drone");
    tabWidget.show();

    std::cout << "Executing application..." << std::endl;
    int result = app.exec();

    std::cout << "Shutting down ROS..." << std::endl;
    rclcpp::shutdown();

    std::cout << "Application exited with code " << result << std::endl;
    return result;
}
