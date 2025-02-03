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

class Init_FlightCommand_length
{
public:
  explicit Init_FlightCommand_length(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::FlightCommand length(::custom_msgs::msg::FlightCommand::_length_type arg)
  {
    msg_.length = std::move(arg);
    return std::move(msg_);
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

class Init_FlightCommand_y
{
public:
  explicit Init_FlightCommand_y(::custom_msgs::msg::FlightCommand & msg)
  : msg_(msg)
  {}
  Init_FlightCommand_altitude y(::custom_msgs::msg::FlightCommand::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_FlightCommand_altitude(msg_);
  }

private:
  ::custom_msgs::msg::FlightCommand msg_;
};

class Init_FlightCommand_x
{
public:
  Init_FlightCommand_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FlightCommand_y x(::custom_msgs::msg::FlightCommand::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_FlightCommand_y(msg_);
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
  return custom_msgs::msg::builder::Init_FlightCommand_x();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__BUILDER_HPP_
