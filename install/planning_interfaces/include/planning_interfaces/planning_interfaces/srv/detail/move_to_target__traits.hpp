// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from planning_interfaces:srv/MoveToTarget.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__TRAITS_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "planning_interfaces/srv/detail/move_to_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace planning_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveToTarget_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToTarget_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace planning_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use planning_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const planning_interfaces::srv::MoveToTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  planning_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use planning_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const planning_interfaces::srv::MoveToTarget_Request & msg)
{
  return planning_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<planning_interfaces::srv::MoveToTarget_Request>()
{
  return "planning_interfaces::srv::MoveToTarget_Request";
}

template<>
inline const char * name<planning_interfaces::srv::MoveToTarget_Request>()
{
  return "planning_interfaces/srv/MoveToTarget_Request";
}

template<>
struct has_fixed_size<planning_interfaces::srv::MoveToTarget_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<planning_interfaces::srv::MoveToTarget_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<planning_interfaces::srv::MoveToTarget_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace planning_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveToTarget_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveToTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveToTarget_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace planning_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use planning_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const planning_interfaces::srv::MoveToTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  planning_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use planning_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const planning_interfaces::srv::MoveToTarget_Response & msg)
{
  return planning_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<planning_interfaces::srv::MoveToTarget_Response>()
{
  return "planning_interfaces::srv::MoveToTarget_Response";
}

template<>
inline const char * name<planning_interfaces::srv::MoveToTarget_Response>()
{
  return "planning_interfaces/srv/MoveToTarget_Response";
}

template<>
struct has_fixed_size<planning_interfaces::srv::MoveToTarget_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<planning_interfaces::srv::MoveToTarget_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<planning_interfaces::srv::MoveToTarget_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<planning_interfaces::srv::MoveToTarget>()
{
  return "planning_interfaces::srv::MoveToTarget";
}

template<>
inline const char * name<planning_interfaces::srv::MoveToTarget>()
{
  return "planning_interfaces/srv/MoveToTarget";
}

template<>
struct has_fixed_size<planning_interfaces::srv::MoveToTarget>
  : std::integral_constant<
    bool,
    has_fixed_size<planning_interfaces::srv::MoveToTarget_Request>::value &&
    has_fixed_size<planning_interfaces::srv::MoveToTarget_Response>::value
  >
{
};

template<>
struct has_bounded_size<planning_interfaces::srv::MoveToTarget>
  : std::integral_constant<
    bool,
    has_bounded_size<planning_interfaces::srv::MoveToTarget_Request>::value &&
    has_bounded_size<planning_interfaces::srv::MoveToTarget_Response>::value
  >
{
};

template<>
struct is_service<planning_interfaces::srv::MoveToTarget>
  : std::true_type
{
};

template<>
struct is_service_request<planning_interfaces::srv::MoveToTarget_Request>
  : std::true_type
{
};

template<>
struct is_service_response<planning_interfaces::srv::MoveToTarget_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__TRAITS_HPP_
