// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/FlightCommand.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/FlightCommand in the package custom_msgs.
typedef struct custom_msgs__msg__FlightCommand
{
  /// Latitude in degrees
  float latitude_deg;
  /// Longitude in degrees
  float longitude_deg;
  /// altitude for drone
  float altitude;
  /// radius if circle/figure8 path
  float radius;
  /// length for square path
  float length;
  /// duration to remain at waypoint
  float duration;
  /// type of waypoint
  int8_t waypoint_type;
} custom_msgs__msg__FlightCommand;

// Struct for a sequence of custom_msgs__msg__FlightCommand.
typedef struct custom_msgs__msg__FlightCommand__Sequence
{
  custom_msgs__msg__FlightCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__FlightCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__FLIGHT_COMMAND__STRUCT_H_
