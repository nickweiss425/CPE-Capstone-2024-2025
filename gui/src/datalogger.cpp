#include "datalogger.hpp"

std::shared_ptr<DataLogger> DataLogger::instance_ = nullptr;
std::mutex DataLogger::instance_mutex_;

/**
 * @brief Create new data logger singleton
 * 
 */
DataLogger::DataLogger()
    : Node("logger_node") {
        recording = false;
    }

/**
 * @brief Destroy the Data Logger:: Data Logger object
 * 
 */
DataLogger::~DataLogger() {
    // Ensure cleanup is performed safely
    close_file();
    cleanup();
}

/**
 * @brief Returns the instance of the DataLogger class.
 * 
 * This function returns a shared pointer to the instance of the DataLogger class. If the instance does not exist,
 * it creates a new instance and returns it.
 * 
 * @return std::shared_ptr<DataLogger> - A shared pointer to the instance of the DataLogger class.
 */
std::shared_ptr<DataLogger> DataLogger::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::shared_ptr<DataLogger>(new DataLogger());
    }
    return instance_;
}

/**
 * @brief Creates a new log file with a timestamp in the filename.
 * 
 * This function closes the current log file (if open), generates a new filename
 * with the current timestamp, and opens the new file for writing. If the file
 * creation is successful, the function writes a header line to the file indicating
 * the creation time.
 * 
 * @note The function uses the current system time to generate the timestamp.
 * 
 * @throws std::system_error if the file creation fails.
 */
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

/**
 * Logs the given data to the log file.
 *
 * @param data_to_log The data to be logged.
 */
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

/**
 * Closes the log file.
 */
void DataLogger::close_file() {
    if (log_file_.is_open()) {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Log file '%s' closed.", file_name_.c_str());
    }
}

/**
 * @brief Get the current recording status
 * 
 * @return true if recording is active
 * @return false if recording is inactive
 */
bool DataLogger::getRecording() {
    return recording;
}

/**
 * @brief Switch the recording status
 * 
 */
void DataLogger::switchRecording() {
    recording = !recording;
}

/**
 * @brief Start the data logging process
 * 
 */
void DataLogger::cleanup() {
    timer_.reset();
    instance_.reset();
}

/**
 * @brief Shuts down the DataLogger instance.
 * 
 * This function is responsible for shutting down the DataLogger instance by closing the file and resetting the instance.
 * 
 * @note This function assumes that the DataLogger instance has been initialized before calling this function.
 */
void DataLogger::shutdown() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance_) {
        instance_->close_file();
        instance_.reset();
    }
}