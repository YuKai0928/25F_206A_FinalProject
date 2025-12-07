// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from planning_interfaces:srv/ContinuousPickPlace.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__TRAITS_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "planning_interfaces/srv/detail/continuous_pick_place__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pick_scan_pose'
// Member 'place_scan_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace planning_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ContinuousPickPlace_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pick_scan_pose
  {
    out << "pick_scan_pose: ";
    to_flow_style_yaml(msg.pick_scan_pose, out);
    out << ", ";
  }

  // member: place_scan_pose
  {
    out << "place_scan_pose: ";
    to_flow_style_yaml(msg.place_scan_pose, out);
    out << ", ";
  }

  // member: pick_distance
  {
    out << "pick_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.pick_distance, out);
    out << ", ";
  }

  // member: retreat_distance
  {
    out << "retreat_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.retreat_distance, out);
    out << ", ";
  }

  // member: place_distance
  {
    out << "place_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.place_distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ContinuousPickPlace_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pick_scan_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pick_scan_pose:\n";
    to_block_style_yaml(msg.pick_scan_pose, out, indentation + 2);
  }

  // member: place_scan_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "place_scan_pose:\n";
    to_block_style_yaml(msg.place_scan_pose, out, indentation + 2);
  }

  // member: pick_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pick_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.pick_distance, out);
    out << "\n";
  }

  // member: retreat_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "retreat_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.retreat_distance, out);
    out << "\n";
  }

  // member: place_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "place_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.place_distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ContinuousPickPlace_Request & msg, bool use_flow_style = false)
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
  const planning_interfaces::srv::ContinuousPickPlace_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  planning_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use planning_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const planning_interfaces::srv::ContinuousPickPlace_Request & msg)
{
  return planning_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<planning_interfaces::srv::ContinuousPickPlace_Request>()
{
  return "planning_interfaces::srv::ContinuousPickPlace_Request";
}

template<>
inline const char * name<planning_interfaces::srv::ContinuousPickPlace_Request>()
{
  return "planning_interfaces/srv/ContinuousPickPlace_Request";
}

template<>
struct has_fixed_size<planning_interfaces::srv::ContinuousPickPlace_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<planning_interfaces::srv::ContinuousPickPlace_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<planning_interfaces::srv::ContinuousPickPlace_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace planning_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ContinuousPickPlace_Response & msg,
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
    out << ", ";
  }

  // member: slides_picked
  {
    out << "slides_picked: ";
    rosidl_generator_traits::value_to_yaml(msg.slides_picked, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ContinuousPickPlace_Response & msg,
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

  // member: slides_picked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slides_picked: ";
    rosidl_generator_traits::value_to_yaml(msg.slides_picked, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ContinuousPickPlace_Response & msg, bool use_flow_style = false)
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
  const planning_interfaces::srv::ContinuousPickPlace_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  planning_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use planning_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const planning_interfaces::srv::ContinuousPickPlace_Response & msg)
{
  return planning_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<planning_interfaces::srv::ContinuousPickPlace_Response>()
{
  return "planning_interfaces::srv::ContinuousPickPlace_Response";
}

template<>
inline const char * name<planning_interfaces::srv::ContinuousPickPlace_Response>()
{
  return "planning_interfaces/srv/ContinuousPickPlace_Response";
}

template<>
struct has_fixed_size<planning_interfaces::srv::ContinuousPickPlace_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<planning_interfaces::srv::ContinuousPickPlace_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<planning_interfaces::srv::ContinuousPickPlace_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<planning_interfaces::srv::ContinuousPickPlace>()
{
  return "planning_interfaces::srv::ContinuousPickPlace";
}

template<>
inline const char * name<planning_interfaces::srv::ContinuousPickPlace>()
{
  return "planning_interfaces/srv/ContinuousPickPlace";
}

template<>
struct has_fixed_size<planning_interfaces::srv::ContinuousPickPlace>
  : std::integral_constant<
    bool,
    has_fixed_size<planning_interfaces::srv::ContinuousPickPlace_Request>::value &&
    has_fixed_size<planning_interfaces::srv::ContinuousPickPlace_Response>::value
  >
{
};

template<>
struct has_bounded_size<planning_interfaces::srv::ContinuousPickPlace>
  : std::integral_constant<
    bool,
    has_bounded_size<planning_interfaces::srv::ContinuousPickPlace_Request>::value &&
    has_bounded_size<planning_interfaces::srv::ContinuousPickPlace_Response>::value
  >
{
};

template<>
struct is_service<planning_interfaces::srv::ContinuousPickPlace>
  : std::true_type
{
};

template<>
struct is_service_request<planning_interfaces::srv::ContinuousPickPlace_Request>
  : std::true_type
{
};

template<>
struct is_service_response<planning_interfaces::srv::ContinuousPickPlace_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__TRAITS_HPP_
