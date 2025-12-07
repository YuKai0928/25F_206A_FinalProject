// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from planning_interfaces:srv/ContinuousPickPlace.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__BUILDER_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "planning_interfaces/srv/detail/continuous_pick_place__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace planning_interfaces
{

namespace srv
{

namespace builder
{

class Init_ContinuousPickPlace_Request_place_distance
{
public:
  explicit Init_ContinuousPickPlace_Request_place_distance(::planning_interfaces::srv::ContinuousPickPlace_Request & msg)
  : msg_(msg)
  {}
  ::planning_interfaces::srv::ContinuousPickPlace_Request place_distance(::planning_interfaces::srv::ContinuousPickPlace_Request::_place_distance_type arg)
  {
    msg_.place_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Request msg_;
};

class Init_ContinuousPickPlace_Request_retreat_distance
{
public:
  explicit Init_ContinuousPickPlace_Request_retreat_distance(::planning_interfaces::srv::ContinuousPickPlace_Request & msg)
  : msg_(msg)
  {}
  Init_ContinuousPickPlace_Request_place_distance retreat_distance(::planning_interfaces::srv::ContinuousPickPlace_Request::_retreat_distance_type arg)
  {
    msg_.retreat_distance = std::move(arg);
    return Init_ContinuousPickPlace_Request_place_distance(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Request msg_;
};

class Init_ContinuousPickPlace_Request_pick_distance
{
public:
  explicit Init_ContinuousPickPlace_Request_pick_distance(::planning_interfaces::srv::ContinuousPickPlace_Request & msg)
  : msg_(msg)
  {}
  Init_ContinuousPickPlace_Request_retreat_distance pick_distance(::planning_interfaces::srv::ContinuousPickPlace_Request::_pick_distance_type arg)
  {
    msg_.pick_distance = std::move(arg);
    return Init_ContinuousPickPlace_Request_retreat_distance(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Request msg_;
};

class Init_ContinuousPickPlace_Request_place_scan_pose
{
public:
  explicit Init_ContinuousPickPlace_Request_place_scan_pose(::planning_interfaces::srv::ContinuousPickPlace_Request & msg)
  : msg_(msg)
  {}
  Init_ContinuousPickPlace_Request_pick_distance place_scan_pose(::planning_interfaces::srv::ContinuousPickPlace_Request::_place_scan_pose_type arg)
  {
    msg_.place_scan_pose = std::move(arg);
    return Init_ContinuousPickPlace_Request_pick_distance(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Request msg_;
};

class Init_ContinuousPickPlace_Request_pick_scan_pose
{
public:
  Init_ContinuousPickPlace_Request_pick_scan_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContinuousPickPlace_Request_place_scan_pose pick_scan_pose(::planning_interfaces::srv::ContinuousPickPlace_Request::_pick_scan_pose_type arg)
  {
    msg_.pick_scan_pose = std::move(arg);
    return Init_ContinuousPickPlace_Request_place_scan_pose(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::planning_interfaces::srv::ContinuousPickPlace_Request>()
{
  return planning_interfaces::srv::builder::Init_ContinuousPickPlace_Request_pick_scan_pose();
}

}  // namespace planning_interfaces


namespace planning_interfaces
{

namespace srv
{

namespace builder
{

class Init_ContinuousPickPlace_Response_slides_picked
{
public:
  explicit Init_ContinuousPickPlace_Response_slides_picked(::planning_interfaces::srv::ContinuousPickPlace_Response & msg)
  : msg_(msg)
  {}
  ::planning_interfaces::srv::ContinuousPickPlace_Response slides_picked(::planning_interfaces::srv::ContinuousPickPlace_Response::_slides_picked_type arg)
  {
    msg_.slides_picked = std::move(arg);
    return std::move(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Response msg_;
};

class Init_ContinuousPickPlace_Response_message
{
public:
  explicit Init_ContinuousPickPlace_Response_message(::planning_interfaces::srv::ContinuousPickPlace_Response & msg)
  : msg_(msg)
  {}
  Init_ContinuousPickPlace_Response_slides_picked message(::planning_interfaces::srv::ContinuousPickPlace_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ContinuousPickPlace_Response_slides_picked(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Response msg_;
};

class Init_ContinuousPickPlace_Response_success
{
public:
  Init_ContinuousPickPlace_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContinuousPickPlace_Response_message success(::planning_interfaces::srv::ContinuousPickPlace_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ContinuousPickPlace_Response_message(msg_);
  }

private:
  ::planning_interfaces::srv::ContinuousPickPlace_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::planning_interfaces::srv::ContinuousPickPlace_Response>()
{
  return planning_interfaces::srv::builder::Init_ContinuousPickPlace_Response_success();
}

}  // namespace planning_interfaces

#endif  // PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__BUILDER_HPP_
