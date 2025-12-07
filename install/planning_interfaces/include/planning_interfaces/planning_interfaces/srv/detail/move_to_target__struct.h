// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from planning_interfaces:srv/MoveToTarget.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__STRUCT_H_
#define PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/MoveToTarget in the package planning_interfaces.
typedef struct planning_interfaces__srv__MoveToTarget_Request
{
  geometry_msgs__msg__PoseStamped target_pose;
} planning_interfaces__srv__MoveToTarget_Request;

// Struct for a sequence of planning_interfaces__srv__MoveToTarget_Request.
typedef struct planning_interfaces__srv__MoveToTarget_Request__Sequence
{
  planning_interfaces__srv__MoveToTarget_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} planning_interfaces__srv__MoveToTarget_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveToTarget in the package planning_interfaces.
typedef struct planning_interfaces__srv__MoveToTarget_Response
{
  bool success;
  rosidl_runtime_c__String message;
} planning_interfaces__srv__MoveToTarget_Response;

// Struct for a sequence of planning_interfaces__srv__MoveToTarget_Response.
typedef struct planning_interfaces__srv__MoveToTarget_Response__Sequence
{
  planning_interfaces__srv__MoveToTarget_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} planning_interfaces__srv__MoveToTarget_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__STRUCT_H_
