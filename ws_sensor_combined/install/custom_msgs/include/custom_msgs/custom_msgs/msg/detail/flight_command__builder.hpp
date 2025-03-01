// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/FlightCommand.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/flight_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_FlightCommand_waypoint_type
{
public:
  explicit Init_FlightCommand_waypoint_type(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::FlightCommand waypoint_type(::custom_msgs::msg::FlightCommand::_waypoint_type_type arg)
  {
    msg_.waypoint_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

class Init_FlightCommand_duration
{
public:
  explicit Init_FlightCommand_duration(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  Init_FlightCommand_waypoint_type duration(::custom_msgs::msg::FlightCommand::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_FlightCommand_waypoint_type(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

class Init_FlightCommand_length
{
public:
  explicit Init_FlightCommand_length(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  Init_FlightCommand_duration length(::custom_msgs::msg::FlightCommand::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_FlightCommand_duration(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

class Init_FlightCommand_radius
{
public:
  explicit Init_FlightCommand_radius(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  Init_FlightCommand_length radius(::custom_msgs::msg::FlightCommand::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_FlightCommand_length(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

class Init_FlightCommand_altitude
{
public:
  explicit Init_FlightCommand_altitude(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  Init_FlightCommand_radius altitude(::custom_msgs::msg::FlightCommand::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_FlightCommand_radius(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

class Init_FlightCommand_longitude_deg
{
public:
  explicit Init_FlightCommand_longitude_deg(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  Init_FlightCommand_altitude longitude_deg(::custom_msgs::msg::FlightCommand::_longitude_deg_type arg)
  {
    msg_.longitude_deg = std::move(arg);
    return Init_FlightCommand_altitude(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

class Init_FlightCommand_latitude_deg
{
public:
  Init_FlightCommand_latitude_deg()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FlightCommand_longitude_deg latitude_deg(::custom_msgs::msg::FlightCommand::_latitude_deg_type arg)
  {
    msg_.latitude_deg = std::move(arg);
    return Init_FlightCommand_longitude_deg(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::FlightCommand>()
{
  return custom_msgs::msg::builder::Init_FlightCommand_latitude_deg();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__BUILDER_HPP_
