// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from planning_interfaces:srv/MoveToTarget.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "planning_interfaces/srv/detail/move_to_target__rosidl_typesupport_introspection_c.h"
#include "planning_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "planning_interfaces/srv/detail/move_to_target__functions.h"
#include "planning_interfaces/srv/detail/move_to_target__struct.h"


// Include directives for member types
// Member `target_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `target_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  planning_interfaces__srv__MoveToTarget_Request__init(message_memory);
}

void planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_fini_function(void * message_memory)
{
  planning_interfaces__srv__MoveToTarget_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_member_array[1] = {
  {
    "target_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(planning_interfaces__srv__MoveToTarget_Request, target_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_members = {
  "planning_interfaces__srv",  // message namespace
  "MoveToTarget_Request",  // message name
  1,  // number of fields
  sizeof(planning_interfaces__srv__MoveToTarget_Request),
  planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_member_array,  // message members
  planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_type_support_handle = {
  0,
  &planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_planning_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_interfaces, srv, MoveToTarget_Request)() {
  planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_type_support_handle.typesupport_identifier) {
    planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &planning_interfaces__srv__MoveToTarget_Request__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "planning_interfaces/srv/detail/move_to_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "planning_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "planning_interfaces/srv/detail/move_to_target__functions.h"
// already included above
// #include "planning_interfaces/srv/detail/move_to_target__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  planning_interfaces__srv__MoveToTarget_Response__init(message_memory);
}

void planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_fini_function(void * message_memory)
{
  planning_interfaces__srv__MoveToTarget_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(planning_interfaces__srv__MoveToTarget_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(planning_interfaces__srv__MoveToTarget_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_members = {
  "planning_interfaces__srv",  // message namespace
  "MoveToTarget_Response",  // message name
  2,  // number of fields
  sizeof(planning_interfaces__srv__MoveToTarget_Response),
  planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_member_array,  // message members
  planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_type_support_handle = {
  0,
  &planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_planning_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_interfaces, srv, MoveToTarget_Response)() {
  if (!planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_type_support_handle.typesupport_identifier) {
    planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &planning_interfaces__srv__MoveToTarget_Response__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "planning_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "planning_interfaces/srv/detail/move_to_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_service_members = {
  "planning_interfaces__srv",  // service namespace
  "MoveToTarget",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_Request_message_type_support_handle,
  NULL  // response message
  // planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_Response_message_type_support_handle
};

static rosidl_service_type_support_t planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_service_type_support_handle = {
  0,
  &planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_interfaces, srv, MoveToTarget_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_interfaces, srv, MoveToTarget_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_planning_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_interfaces, srv, MoveToTarget)() {
  if (!planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_service_type_support_handle.typesupport_identifier) {
    planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_interfaces, srv, MoveToTarget_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_interfaces, srv, MoveToTarget_Response)()->data;
  }

  return &planning_interfaces__srv__detail__move_to_target__rosidl_typesupport_introspection_c__MoveToTarget_service_type_support_handle;
}
