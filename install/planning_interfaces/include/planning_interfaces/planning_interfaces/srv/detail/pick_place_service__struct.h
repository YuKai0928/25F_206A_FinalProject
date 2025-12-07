// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from planning_interfaces:srv/PickPlaceService.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__STRUCT_H_
#define PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pick_pose'
// Member 'place_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/PickPlaceService in the package planning_interfaces.
typedef struct planning_interfaces__srv__PickPlaceService_Request
{
  geometry_msgs__msg__PoseStamped pick_pose;
  geometry_msgs__msg__PoseStamped place_pose;
} planning_interfaces__srv__PickPlaceService_Request;

// Struct for a sequence of planning_interfaces__srv__PickPlaceService_Request.
typedef struct planning_interfaces__srv__PickPlaceService_Request__Sequence
{
  planning_interfaces__srv__PickPlaceService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} planning_interfaces__srv__PickPlaceService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PickPlaceService in the package planning_interfaces.
typedef struct planning_interfaces__srv__PickPlaceService_Response
{
  bool success;
  rosidl_runtime_c__String message;
} planning_interfaces__srv__PickPlaceService_Response;

// Struct for a sequence of planning_interfaces__srv__PickPlaceService_Response.
typedef struct planning_interfaces__srv__PickPlaceService_Response__Sequence
{
  planning_interfaces__srv__PickPlaceService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} planning_interfaces__srv__PickPlaceService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__STRUCT_H_
