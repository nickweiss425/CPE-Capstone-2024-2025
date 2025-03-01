// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/FlightCommand.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__FlightCommand __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__FlightCommand __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FlightCommand_
{
  using Type = FlightCommand_<ContainerAllocator>;

  explicit FlightCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude_deg = 0.0f;
      this->longitude_deg = 0.0f;
      this->altitude = 0.0f;
      this->radius = 0.0f;
      this->length = 0.0f;
      this->duration = 0.0f;
      this->waypoint_type = 0;
    }
  }

  explicit FlightCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude_deg = 0.0f;
      this->longitude_deg = 0.0f;
      this->altitude = 0.0f;
      this->radius = 0.0f;
      this->length = 0.0f;
      this->duration = 0.0f;
      this->waypoint_type = 0;
    }
  }

  // field types and members
  using _latitude_deg_type =
    float;
  _latitude_deg_type latitude_deg;
  using _longitude_deg_type =
    float;
  _longitude_deg_type longitude_deg;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _radius_type =
    float;
  _radius_type radius;
  using _length_type =
    float;
  _length_type length;
  using _duration_type =
    float;
  _duration_type duration;
  using _waypoint_type_type =
    int8_t;
  _waypoint_type_type waypoint_type;

  // setters for named parameter idiom
  Type & set__latitude_deg(
    const float & _arg)
  {
    this->latitude_deg = _arg;
    return *this;
  }
  Type & set__longitude_deg(
    const float & _arg)
  {
    this->longitude_deg = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__radius(
    const float & _arg)
  {
    this->radius = _arg;
    return *this;
  }
  Type & set__length(
    const float & _arg)
  {
    this->length = _arg;
    return *this;
  }
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__waypoint_type(
    const int8_t & _arg)
  {
    this->waypoint_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::FlightCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::FlightCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::FlightCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::FlightCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__FlightCommand
    std::shared_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__FlightCommand
    std::shared_ptr<custom_msgs::msg::FlightCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FlightCommand_ & other) const
  {
    if (this->latitude_deg != other.latitude_deg) {
      return false;
    }
    if (this->longitude_deg != other.longitude_deg) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->radius != other.radius) {
      return false;
    }
    if (this->length != other.length) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->waypoint_type != other.waypoint_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const FlightCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FlightCommand_

// alias to use template instance with default allocator
using FlightCommand =
  custom_msgs::msg::FlightCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__STRUCT_HPP_
