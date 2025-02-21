#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <mutex>

class DataLogger : public rclcpp::Node {
    
public:
    DataLogger();
    ~DataLogger();

    static std::shared_ptr<DataLogger> getInstance(); // Get the singleton instance of DataLogger
    static void shutdown(); // Explicitly shutdown the logger
    void create_file(); // Create txt logger file
    void log_data(const std::string& data_to_log); // Function to write to logger file
    void close_file(); // Close file
    void cleanup(); // Clean up singleton
    bool getRecording(); // Get recording variable
    void switchRecording(); // Toggle recording

private:
    bool recording; // Recording data to logger file 
    DataLogger(const DataLogger&) = delete; // Disable copy and assignment
    DataLogger& operator=(const DataLogger&) = delete;
    static std::shared_ptr<DataLogger> instance_; // The singleton instance
    static std::mutex instance_mutex_; // Mutex for thread safety
    std::ofstream log_file_; // Log file stream
    std::string file_name_; // Current log file name
    rclcpp::TimerBase::SharedPtr timer_; // ROS 2 timer
};
