#include "datalogger.hpp"

std::shared_ptr<DataLogger> DataLogger::instance_ = nullptr;
std::mutex DataLogger::instance_mutex_;

DataLogger::DataLogger()
    : Node("logger_node") {
        recording = false;
    }

DataLogger::~DataLogger() {
    // Ensure cleanup is performed safely
    close_file();
    cleanup();
}

std::shared_ptr<DataLogger> DataLogger::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::shared_ptr<DataLogger>(new DataLogger());
    }
    return instance_;
}

void DataLogger::create_file() {
    if (log_file_.is_open()) {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Log file '%s' closed.", file_name_.c_str());
    }

    auto now = this->get_clock()->now();
    std::time_t now_time_t = static_cast<std::time_t>(now.seconds());
    std::tm now_tm = *std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << "output_"
        << std::put_time(&now_tm, "%Y_%m_%d_%H_%M_%S")
        << ".txt";

    file_name_ = oss.str();
    log_file_.open(file_name_, std::ios::out);

    if (log_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "File '%s' created successfully.", file_name_.c_str());
        log_file_ << "Log file created by DataLogger at "
                  << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << "\n";
        log_file_.flush();
    } else {
        std::error_code ec(errno, std::generic_category());
        RCLCPP_ERROR(this->get_logger(), "Failed to create the file '%s'. Error: %s", file_name_.c_str(), ec.message().c_str());
    }
}

void DataLogger::log_data(const std::string& data_to_log) {
    if (!log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Log file is not open. Cannot log data.");
        return;
    }

    auto now = this->get_clock()->now();
    std::time_t now_time_t = static_cast<std::time_t>(now.seconds());
    std::tm now_tm = *std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%H_%M_%S") << " : " << data_to_log << "\n";
    std::string log_entry = oss.str();

    log_file_ << log_entry;
    log_file_.flush();
    RCLCPP_INFO(this->get_logger(), "Logged data: %s", log_entry.c_str());
}

void DataLogger::close_file() {
    if (log_file_.is_open()) {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Log file '%s' closed.", file_name_.c_str());
    }
}

bool DataLogger::getRecording() {
    return recording;
}

void DataLogger::switchRecording() {
    recording = !recording;
}

void DataLogger::cleanup() {
    timer_.reset();
    instance_.reset();
}

void DataLogger::shutdown() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance_) {
        instance_->close_file();
        instance_.reset();
    }
}