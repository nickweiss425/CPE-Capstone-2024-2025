
/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 */


 #include <px4_msgs/msg/offboard_control_mode.hpp>
 #include <px4_msgs/msg/trajectory_setpoint.hpp>
 #include <px4_msgs/msg/position_setpoint.hpp>
 #include <px4_msgs/msg/position_setpoint_triplet.hpp>
 #include <px4_msgs/msg/sensor_gps.hpp>
 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <px4_msgs/msg/sensor_combined.hpp>
 #include <px4_msgs/msg/vehicle_control_mode.hpp>
 #include <px4_msgs/msg/vehicle_local_position.hpp>
 #include <px4_msgs/msg/vehicle_odometry.hpp>
 #include <px4_msgs/msg/home_position.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include "std_msgs/msg/int32.hpp"
 #include <stdint.h>
 #include <sensor_msgs/msg/nav_sat_fix.hpp>
 #include <sensor_msgs/msg/imu.hpp>
 #include <gui_messages/msg/flight_command.hpp>
 
 
 
 
 #include <chrono>
 #include <iostream>
 #include <cmath>
 
 
 #define EARTH_RADIUS 6378137.0 // Earth's radius in meters
 
 
 using namespace std::chrono;
 using namespace std::chrono_literals;
 using namespace px4_msgs::msg;
 using namespace gui_messages::msg;
  
  
  
  
  
  
  
  
  
 class OffboardControl : public rclcpp::Node // new class that inherits from ROS 2 node class
 {
 public:
 
 
 
 
	 OffboardControl() : Node("drone_launch"), current_state_(FlightState::LANDED) // initialize node (base class constructor)
	 {
		 rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		 auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
 
 
		 // create_publisher is a method of rclcpp::Node that creates a publisher for a specific topic.
		 // 10 is the queue size for messages
		 offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		 trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		 vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		 gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
		 imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
		 command_ack_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/waypoint_ack", 10);
 
 
		 // subscriber to monitor local location of the drone
		 odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
			 "/fmu/out/vehicle_odometry", qos, std::bind(&OffboardControl::odometryCallback, this, std::placeholders::_1));
 
 
		 // subscriber to monitor gps location of drone
		 gps_px4_subscriber_ = this->create_subscription<SensorGps>(
			 "/fmu/out/vehicle_gps_position",  qos, std::bind(&OffboardControl::gpsCallback, this, std::placeholders::_1));
	 
		 // subscriber to sensor information from the drone
		 sensor_px4_subscriber_ = this->create_subscription<SensorCombined>(
			 "/fmu/out/sensor_combined",  qos, std::bind(&OffboardControl::sensorCallback, this, std::placeholders::_1));
 
 
		 // subscriber to read desired state messages from gui
		 flight_command_subscriber_ = this->create_subscription<FlightCommand>(
			 "flight_command", 10, std::bind(&OffboardControl::flightCommandCallback, this, std::placeholders::_1));
 
 
 
		 offboard_setpoint_counter_ = 0;
		 gui_data_delay_ = 0;
 
 
		 // timer to ensure this block of code runs every 100ms
		 auto timer_callback = [this]() -> void {
		 
			 if (offboard_setpoint_counter_ == 10) {
				 // Change to Offboard mode after 10 setpoints
				 this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			 
				 // delay data publishing to 1 time out of 10 callbacks (1 sec)
				 gui_data_delay_++;
				 if (gui_data_delay_ == 10){
					 publishSensorData();
					 publishGPSData();
					 gui_data_delay_ = 0;
				 }
 
 
				 // check current command state from GUI
				 //RCLCPP_INFO(this->get_logger(), "CURRENT STATE: %s", flightStateToString(static_cast<int>(current_state_)).c_str());
				 switch (current_state_) {
					 // landed state, no work to do
					 case FlightState::LANDED:
						 break;
					 // fly straight up from home to given altitude
					 case FlightState::TAKEOFF:
						 handleTakeoffState(-10);
						 break;
					 // land at set home position: local position of 0,0,0
					 case FlightState::HOME_LAND:
						 handleHomeLandingState();                    
						 break;
					 case FlightState::LOITER_WAYPOINT:
						 handleLoiterWaypointState(target_x_, target_y_, target_altitude_);
						 break;
					 case FlightState::LOITER:
						 break;
					 // fly to edge of circle, centered around given waypoint
					 case FlightState::CIRCLE_WAYPOINT:
						 handleCircleWaypointState(target_x_, target_y_, target_altitude_, circle_radius_);
						 break;
					 // fly in a circle path around center point with given radius
					 case FlightState::CIRCLE_PATH:
						 handleCirclePathState(target_x_, target_y_, target_altitude_, circle_radius_);
						 break;
					 // fly to edge of square, centered around given waypoint
					 case FlightState::SQUARE_WAYPOINT:
						 handleSquareWaypointState(target_x_, target_y_, target_altitude_, circle_radius_);
						 break;
					 // fly in a square path around center point with given length
					 case FlightState::SQUARE_PATH:
						 handleSquarePathState();
						 break;
					 // fly to center of figure 8
					 case FlightState::FIGURE8_WAYPOINT:
						 handleFigure8WaypointState(target_x_, target_y_ , target_altitude_, circle_radius_);
						 break;
					 // fly in a figure 8 pattern centered around waypoint with given radius
					 case FlightState::FIGURE8_PATH:
						 handleFigure8PathState(target_x_, target_y_ , target_altitude_, circle_radius_);
						 break;
					 // land directly below where drone currently is
					 case FlightState::LAND_IN_PLACE:
						 handleLandInPlaceState();
						 break;
				  }
			  }
  
  
			 // offboard_control_mode needs to be paired with trajectory_setpoint
			 publish_offboard_control_mode();
 
 
 
 
			 // stop the counter after reaching 11
			 if (offboard_setpoint_counter_ < 10) {
				 offboard_setpoint_counter_++;
			 }
		 };
		 timer_ = this->create_wall_timer(100ms, timer_callback);
	 }
 
 
	 void arm();
	 void disarm();
  
  
 private:
	 // enum to hold flight states for state machine
	 enum class FlightState {
		 LANDED = 0,
		 TAKEOFF = 1,
		 HOME_LAND = 2,
		 CIRCLE_WAYPOINT = 3,
		 CIRCLE_PATH = 4,
		 LOITER_WAYPOINT = 5,
		 LOITER = 6,
		 SQUARE_WAYPOINT = 7,
		 SQUARE_PATH = 8,
		 LAND_IN_PLACE = 9,
		 FIGURE8_WAYPOINT = 10,
		 FIGURE8_PATH = 11
	 };
 
 
	 FlightState current_state_;
 
 
	 rclcpp::TimerBase::SharedPtr timer_;
 
 
	 rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	 rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	 rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	 rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
	 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
	 rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_ack_publisher_;
 
 
 
 
	 //rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gui_command_subscriber_;
	 rclcpp::Subscription<SensorGps>::SharedPtr gps_px4_subscriber_;
	 rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscriber_;
	 rclcpp::Subscription<SensorCombined>::SharedPtr sensor_px4_subscriber_;
	 rclcpp::Subscription<FlightCommand>::SharedPtr flight_command_subscriber_;
  
  
  
  
	 std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
 
 
	 uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
 
 
	 // counter to ensure a 1 second interval between data passed to the gui
	 uint64_t gui_data_delay_;
 
 
	 //
	 bool armed = false;
	 float home_lat_;
	 float home_lon_;
 
 
	 // in class variables to hold local position of drone
	 float cur_x_;
	 float cur_y_;
	 float cur_z_;
 
 
	 // in class variables to hold angular velocity of drone
	 float angular_velocity_x_;
	 float angular_velocity_y_;
	 float angular_velocity_z_;
 
 
	 // in class variables to hold linear acceleration of drone
	 float linear_acceleration_x_;
	 float linear_acceleration_y_;
	 float linear_acceleration_z_;
 
 
	 // in class variables to hold current target local positions
	 float target_x_;
	 float target_y_;
	 float target_z_;
	 float target_altitude_ = -10;
 
 
	 // in class variables to hold current target gps positions
	 float target_lat_;
	 float target_lon_;
 
 
	 // parameters to help with square path
	 float square_length_;
	 uint8_t square_cur_edge_;
	 float cur_square_x_;
	 float cur_square_y_;
	 float square_step_;
	 float steps_per_edge_ = 250;
	 int steps_taken_;
 
 
	 // parameters to help with circle and figure 8 path
	 float circle_radius_;
	 float angle_;
	 float angular_step_ = 0.01;
	 float figure8_y;
 
 
	 // in class variables to hold current global position
	 float cur_lat_;
	 float cur_lon_;
	 float cur_altitude_;
 
 
 
 
	 void publish_offboard_control_mode();
	 void publish_trajectory_setpoint(float x, float y, float z);
	 void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
 
 
 
	 // helper function to detect if the drone is within acceptable range of waypoint
	 bool check_at_setpoint(float target_x, float target_y, float target_z, float accepted_range){
		 float distance = std::sqrt(std::pow(target_x - cur_x_, 2) + std::pow(target_y - cur_y_, 2) + std::pow(target_z - cur_z_, 2));
		 if (distance < accepted_range){
			 return true;
		 }
		 else{
			 return false;
		 }
 
 
	 }
 
 
	 // helper function to handle launching the drone to given altitude
	 void handleTakeoffState(float initial_altitude)
	 {
		 // Arm the vehicle
		 this->arm();
		 if (!check_at_setpoint(0, 0, initial_altitude, 0.2)){
			 publish_trajectory_setpoint(0, 0, initial_altitude);
		 }
		 // send ack once reached takeoff location
		 else{
			 RCLCPP_INFO(this->get_logger(), "PUBLISHING TAKEOFF ACK\n");
			 auto ack_msg = std_msgs::msg::Int32();
			 ack_msg.data = static_cast<int>(FlightState::TAKEOFF);
			 command_ack_publisher_->publish(ack_msg);
		 }
 
	 }
  
 
	 // helper function to handle returning the drone to home position
	 void handleLandInPlaceState(void)
	 {
		 if (!check_at_setpoint(cur_x_, cur_y_, 0.0, 0.2)){
			 publish_trajectory_setpoint(cur_x_, cur_y_, 0.0);
		 }
		 else{
			 this->disarm();
			 current_state_ = FlightState::LANDED;
		 }
 
	 }
  
	 // helper function to handle returning the drone to home position
	 void handleHomeLandingState(void)
	 {
		 if (!check_at_setpoint(0.0, 0.0, 0.0, 0.2)){
			 publish_trajectory_setpoint(0.0, 0.0, 0.0);
		 }
		 else{
			 this->disarm();
			 current_state_ = FlightState::LANDED;
		 }
 
	 }
 
 
	 // keep sending trajectory setpoint messages until at target setpoint
	 void handleLoiterWaypointState(float x, float y, float z)
	 {  
		 if (!check_at_setpoint(x, y, z, 0.2)){
			 publish_trajectory_setpoint(x, y, z);
		 }
		 else{
			 // send ACK to gui
			 RCLCPP_INFO(this->get_logger(), "PUBLISHING LOITER ACK\n");
			 auto ack_msg = std_msgs::msg::Int32();
			 ack_msg.data = static_cast<int>(FlightState::LOITER);
			 command_ack_publisher_->publish(ack_msg);
			 current_state_ = FlightState::LOITER;
		 }
	 }
 
 
 
 
	 // start square path by going to the center waypoint
	 void handleFigure8WaypointState(float waypoint_x, float waypoint_y, float waypoint_z, float radius){
		 // go to center of figure 8 with a tolerance of 0.2 meters
		 if (!check_at_setpoint(waypoint_x, waypoint_y, waypoint_z, 0.2)){
			 publish_trajectory_setpoint(waypoint_x, waypoint_y, waypoint_z);
		 }
		 else{
			 // after reaching center, start right part of 8
			 figure8_y = waypoint_y + radius;
 
 
			 // starting angle = 1.5pi to ensure smooth path from center to circle
			 angle_ = 1.5 * M_PI;
 
 
			 // send ACK to gui
			 RCLCPP_INFO(this->get_logger(), "PUBLISHING FIGURE8 ACK\n");
			 auto ack_msg = std_msgs::msg::Int32();
			 ack_msg.data = static_cast<int>(FlightState::FIGURE8_PATH);
			 command_ack_publisher_->publish(ack_msg);
 
 
			 // start flying on path
			 current_state_ = FlightState::FIGURE8_PATH;
		 }
 
 
	 }
 
 
 
 
	 void handleFigure8PathState(float waypoint_x, float waypoint_y, float altitude, float radius){
		 // calculate next x and y using trig
		 double x = waypoint_x + radius * std::cos(angle_);
		 double y = figure8_y + radius * std::sin(angle_);
 
 
		 // if on right part of 8, increase angle
		 if (figure8_y > waypoint_y){
			 angle_ += angular_step_;
		 }
		 // if on left part of 8
		 else if (figure8_y < waypoint_y){
			 angle_ -= angular_step_;
		 }
	 
 
 
		 // if on right part of 8 and drone comes back to start, change to left part of 8
		 if (figure8_y > waypoint_y && angle_ > (3.5 * M_PI)){
			 figure8_y = waypoint_y - radius;
			 angle_ = 0.5 * M_PI;
		 }
		 // if on left part of 8 and drone comes back to start, change to right part of 8
		 else if (figure8_y < waypoint_y && angle_ < -1.5 * M_PI){
			 figure8_y = waypoint_y + radius;
			 angle_ = 1.5 * M_PI;
		 }
 
 
		 // send drone to new coords
		 publish_trajectory_setpoint(x, y, altitude);
	 }
 
 
 
 
	 // move to given setpoint and go to outside edge of circle
	 void handleCircleWaypointState(float waypoint_x, float waypoint_y, float waypoint_z, float radius){
		 float edge_x = waypoint_x + radius;
		 // Arm the vehicle
		 this->arm();
		 //RCLCPP_INFO(this->get_logger(), "Starting Point: (%f, %d)", edge_x, waypoint_y);
		 // keep publishing trajectory setpoint messages until at target location
		 if (!check_at_setpoint(edge_x, waypoint_y, waypoint_z, 0.2)){
			 publish_trajectory_setpoint(edge_x, waypoint_y, waypoint_z);
		 }
		 else{
			 // once reached target waypoint, start circular path
			 angle_ = 0;
 
 
			 // send ACK to gui
			 RCLCPP_INFO(this->get_logger(), "PUBLISHING CIRCLE ACK\n");
			 auto ack_msg = std_msgs::msg::Int32();
			 ack_msg.data = static_cast<int>(FlightState::CIRCLE_PATH);
			 command_ack_publisher_->publish(ack_msg);
					 
 
 
			 current_state_ = FlightState::CIRCLE_PATH;
		 }
	 }
 
 
	 // keep the drone moving in a circle of given radius
	 void handleCirclePathState(float center_x, float center_y, float altitude, float radius){
		 // Arm the vehicle
		 this->arm();
 
 
		 // calculate next x and y using trig
		 double x = center_x + radius * std::cos(angle_);
		 double y = center_y + radius * std::sin(angle_);
 
 
		 // Calculate and publish circular trajectory setpoints
		 angle_ += angular_step_;
 
 
		 //Keep angle within [0, 2*PI]
		 if (angle_ >= 2 * M_PI) {
			 angle_ -= 2 * M_PI;
		 }
 
 
		 publish_trajectory_setpoint(x, y, altitude);
	 }
 
 
 
 
 
 
	 void handleSquareWaypointState(float waypoint_x, float waypoint_y, float waypoint_z, float square_length){
		 float edge_x = waypoint_x + (0.5 * square_length);
		 float edge_y = waypoint_y - (0.5 * square_length);
 
 
		 // Arm the vehicle
		 this->arm();
 
 
		 // keep publishing trajectory setpoint messages until at target location
		 if (!check_at_setpoint(edge_x, edge_y, waypoint_z, 0.3)){
			 publish_trajectory_setpoint(edge_x, edge_y, waypoint_z);
		 }
		 else{
			 // once reached target waypoint, start square path
			 square_cur_edge_ = 1;
			 square_step_ = square_length / steps_per_edge_;
			 steps_taken_ = 0;
			 cur_square_x_ = edge_x;
			 cur_square_y_ = edge_y;
 
 
			 // send ACK to gui
			 RCLCPP_INFO(this->get_logger(), "PUBLISHING SQUARE ACK\n");
			 auto ack_msg = std_msgs::msg::Int32();
			 ack_msg.data = static_cast<int>(FlightState::SQUARE_PATH);
			 command_ack_publisher_->publish(ack_msg);
					 
 
 
			 current_state_ = FlightState::SQUARE_PATH;
		 }
	 }
 
 
	 // creates square path for drone to fly in
	 void handleSquarePathState(void){
		 this->arm();
		 // left edge of square
		 if (square_cur_edge_ == 1){
			 // increment y coords by step
			 cur_square_y_ += square_step_;
			 publish_trajectory_setpoint(cur_square_x_, cur_square_y_, target_altitude_);
			 steps_taken_++;
			 // change to top edge once number of steps reached
			 if (steps_taken_ == steps_per_edge_){
				 steps_taken_ = 0;
				 square_cur_edge_ = 2;
			 }
 
 
		 }
		 // top edge of square
		 else if (square_cur_edge_ == 2){
			 // decrement x coords by step
			 cur_square_x_ -= square_step_;
			 publish_trajectory_setpoint(cur_square_x_, cur_square_y_, target_altitude_);
			 steps_taken_++;
			 // change to right edge once number of steps reached
			 if (steps_taken_ == steps_per_edge_){
				 steps_taken_ = 0;
				 square_cur_edge_ = 3;
			 }
 
 
		 }
		 // right edge of square
		 else if (square_cur_edge_ == 3){
			 // decrement y coords by step
			 cur_square_y_ -= square_step_;
			 publish_trajectory_setpoint(cur_square_x_, cur_square_y_, target_altitude_);
			 steps_taken_++;
			 // change to bottom edge once number of steps reached
			 if (steps_taken_ == steps_per_edge_){
				 steps_taken_ = 0;
				 square_cur_edge_ = 4;
			 }
 
 
		 }
		 // bottom edge of square
		 else if (square_cur_edge_ == 4){
			 // increment x coords by step
			 cur_square_x_ += square_step_;
			 publish_trajectory_setpoint(cur_square_x_, cur_square_y_, target_altitude_);
			 steps_taken_++;
			 // change to left edge once number of steps reached
			 if (steps_taken_ == steps_per_edge_){
				 steps_taken_ = 0;
				 square_cur_edge_ = 1;
			 }
 
 
		 }
	 }
 
 
 
	 // callback function for when drone receives odometry messages from uorb topic
	 void odometryCallback(const VehicleOdometry msg) {
		 cur_x_ = msg.position[0];
		 cur_y_ = msg.position[1];
		 cur_z_ = msg.position[2];
	 }
 
 
 
 
	 // callback to set global parameters for requested waypoint from GUI
	 void flightCommandCallback(const FlightCommand msg){
		 target_lon_ = msg.longitude_deg;
		 target_lat_ = msg.latitude_deg;
		 target_altitude_ = -1 * msg.altitude;
		 circle_radius_ = msg.radius;
		 square_length_ = msg.length;
		 current_state_ = static_cast<FlightState>(msg.waypoint_type);
		 updateLocalTargets();
		 std::cout << "Received Flight Command:\n" << "Latitude: " << target_lat_ << "\nLongitude: " << target_lon_ << "\nAltitude: " << target_altitude_ << "\nRadius: " << circle_radius_ <<  "\nLength: " << square_length_ << std::endl;
	 
	 }
 
 
	 // Convert degrees to radians
	 float deg2Rad(float degrees) {
		 return degrees * (M_PI / 180.0f);
	 }
 
 
	 void updateLocalTargets(void) {
		 float dLon = deg2Rad(target_lon_ - home_lon_);
		 float dLat = deg2Rad(target_lat_ - home_lat_);
 
 
		 target_x_ = dLat * static_cast<float>(EARTH_RADIUS);
		 target_y_ = dLon * static_cast<float>(EARTH_RADIUS) * std::cos(deg2Rad(home_lat_));
	 }
 
 
 
 
 
 
	 // callback function for when drone receives gps messages
	 void gpsCallback(const SensorGps msg) {
	 
		 // update in class variables
		 cur_lat_ = msg.latitude_deg;
		 cur_lon_ = msg.longitude_deg;
		 cur_altitude_ = msg.altitude_msl_m;
		 RCLCPP_INFO(this->get_logger(), "(%f, %f)\n", cur_lat_, cur_lon_);
	 }
 
 
	 // publish drone's current gps coords to gui
	 void publishGPSData(void){
		 auto message = sensor_msgs::msg::NavSatFix();
		 message.latitude = cur_lat_;  
		 message.longitude = cur_lon_;
		 message.altitude = cur_altitude_;  
 
 
		 message.header.stamp = this->now();
		 message.header.frame_id = "/gps/fix";
 
 
		 gps_publisher_->publish(message);
	 }
 
 
 
 
	 // callback to pull data from PX4 sensor topic
	 void sensorCallback(const SensorCombined msg) {
		 angular_velocity_x_ = msg.gyro_rad[0];
		 angular_velocity_y_ = msg.gyro_rad[1];
		 angular_velocity_z_ = msg.gyro_rad[2];
 
 
		 linear_acceleration_x_ = msg.accelerometer_m_s2[0];
		 linear_acceleration_y_ = msg.accelerometer_m_s2[1];
		 linear_acceleration_z_ = msg.accelerometer_m_s2[2];
	 }
 
 
	 // publish current sensor data from drone to defined
	 void publishSensorData(void){
		 auto imu_msg = sensor_msgs::msg::Imu();
		 imu_msg.header.stamp = this->now();
		 imu_msg.header.frame_id = "imu_frame";
 
 
		 imu_msg.angular_velocity.x = angular_velocity_x_;
		 imu_msg.angular_velocity.y = angular_velocity_y_;
		 imu_msg.angular_velocity.z = angular_velocity_z_;
 
 
		 imu_msg.linear_acceleration.x = linear_acceleration_x_;
		 imu_msg.linear_acceleration.y = linear_acceleration_y_;
		 imu_msg.linear_acceleration.z = linear_acceleration_z_;
 
 
		 imu_publisher_->publish(imu_msg);
	 }
 
 
 
 
 
 
	 // convert state int into appropriate string for logging purposes
	 std::string flightStateToString(int state_int) {
		 switch (static_cast<FlightState>(state_int)) {
			 case FlightState::LANDED:
				 return "LANDED";
				 break;
			 case FlightState::TAKEOFF:
				 return "TAKEOFF";
				 break;
			 case FlightState::HOME_LAND:
				 return "HOME_LAND";
				 break;
			 case FlightState::CIRCLE_WAYPOINT:
				 return "CIRCLE_WAYPOINT";
				 break;
			 case FlightState::CIRCLE_PATH:
				 return "CIRCLE_PATH";
				 break;
			 case FlightState::LOITER_WAYPOINT:
				 return "LOITER_WAYPOINT";
				 break;
			 case FlightState::LOITER:
				 return "LOITER";
				 break;
			 case FlightState::SQUARE_WAYPOINT:
				 return "SQUARE_WAYPOINT";
				 break;
			 case FlightState::SQUARE_PATH:
				 return "SQUARE_PATH";
				 break;
			 case FlightState::LAND_IN_PLACE:
				 return "LAND_IN_PLACE";
				 break;
			 case FlightState::FIGURE8_WAYPOINT:
				 return "FIGURE8_WAYPOINT";
				 break;
			 case FlightState::FIGURE8_PATH:
				 return "FIGURE8_PATH";
				 break;
			 default:
				 return "UNKNOWN";
			 }
	 }
 };
  
  
  
  
  
  
 /**
  * @brief Send a command to Arm the vehicle
  */
 void OffboardControl::arm()
 {
	 if (armed == false){
		 home_lat_ = cur_lat_;
		 home_lon_ = cur_lon_;
		 armed = true;
		 RCLCPP_INFO(this->get_logger(), "Home Latitude: %f\nHome Longitude: %f\n", home_lat_, home_lon_);
	 }
	 publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
 
 
	 //RCLCPP_INFO(this->get_logger(), "Arm command send");
 }
  
  
 /**
  * @brief Send a command to Disarm the vehicle
  */
 void OffboardControl::disarm()
 {
	 publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
 
 
	 //RCLCPP_INFO(this->get_logger(), "Disarm command send");
 }
  
  
 /**
  * @brief Publish the offboard control mode.
  *        For this example, only position and altitude controls are active.
  */
 void OffboardControl::publish_offboard_control_mode()
 {
	 OffboardControlMode msg{};
	 msg.position = true;
	 msg.velocity = false;
	 msg.acceleration = false;
	 msg.attitude = false;
	 msg.body_rate = false;
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	 offboard_control_mode_publisher_->publish(msg);
 }
  
  
 /**
  * @brief Publish a trajectory setpoint
  *        For this example, it sends a trajectory setpoint to make the
  *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
  */
 void OffboardControl::publish_trajectory_setpoint(float x, float y, float z)
 {
	 TrajectorySetpoint msg{};
	 msg.position = {x, y, z};
	 msg.yaw = -3.14; // [-PI:PI]
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	 trajectory_setpoint_publisher_->publish(msg);
 }
  
  
  
  
  
  
  
  
 /**
  * @brief Publish vehicle commands
  * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
  * @param param1    Command parameter 1
  * @param param2    Command parameter 2
  */
 void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
 {
	 VehicleCommand msg{};
	 msg.param1 = param1;
	 msg.param2 = param2;
	 msg.command = command;
	 msg.target_system = 1;
	 msg.target_component = 1;
	 msg.source_system = 1;
	 msg.source_component = 1;
	 msg.from_external = true;
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	 vehicle_command_publisher_->publish(msg);
 }
 
 
 int main(int argc, char *argv[])
 {
	 std::cout << "Starting offboard control node..." << std::endl;
	 setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	 rclcpp::init(argc, argv);
	 rclcpp::spin(std::make_shared<OffboardControl>());
 
 
	 rclcpp::shutdown();
	 return 0;
 }
  
  
  
  
 