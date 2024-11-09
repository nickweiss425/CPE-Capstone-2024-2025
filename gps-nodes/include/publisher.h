#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public QObject, public rclcpp::Node {
    Q_OBJECT

	public:
    		MinimalPublisher();

	signals:
`  		void coordinatesUpdated(double latitude, double longitude, double altitude);

	private:
    		void timer_callback();
    		rclcpp::TimerBase::SharedPtr timer_;
    		rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    		size_t count_;
		double x_;
};

#endif
