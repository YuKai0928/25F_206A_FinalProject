// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from planning_interfaces:srv/ContinuousPickPlace.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__STRUCT_H_
#define PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pick_scan_pose'
// Member 'place_scan_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/ContinuousPickPlace in the package planning_interfaces.
typedef struct planning_interfaces__srv__ContinuousPickPlace_Request
{
  /// Input: Scan poses where camera can see targets
  /// Flange pose to see slide tray
  geometry_msgs__msg__PoseStamped pick_scan_pose;
  /// Flange pose to see target plate
  geometry_msgs__msg__PoseStamped place_scan_pose;
  /// Tunable parameters (in meters)
  /// Distance to lower for grasp
  float pick_distance;
  /// Distance to lift after grasp
  float retreat_distance;
  /// Distance to lower for place
  float place_distance;
} planning_interfaces__srv__ContinuousPickPlace_Request;

// Struct for a sequence of planning_interfaces__srv__ContinuousPickPlace_Request.
typedef struct planning_interfaces__srv__ContinuousPickPlace_Request__Sequence
{
  planning_interfaces__srv__ContinuousPickPlace_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} planning_interfaces__srv__ContinuousPickPlace_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ContinuousPickPlace in the package planning_interfaces.
typedef struct planning_interfaces__srv__ContinuousPickPlace_Response
{
  bool success;
  rosidl_runtime_c__String message;
  int32_t slides_picked;
} planning_interfaces__srv__ContinuousPickPlace_Response;

// Struct for a sequence of planning_interfaces__srv__ContinuousPickPlace_Response.
typedef struct planning_interfaces__srv__ContinuousPickPlace_Response__Sequence
{
  planning_interfaces__srv__ContinuousPickPlace_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} planning_interfaces__srv__ContinuousPickPlace_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__STRUCT_H_
