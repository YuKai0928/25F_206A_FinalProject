// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from planning_interfaces:srv/PickPlaceService.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__BUILDER_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "planning_interfaces/srv/detail/pick_place_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace planning_interfaces
{

namespace srv
{

namespace builder
{

class Init_PickPlaceService_Request_place_pose
{
public:
  explicit Init_PickPlaceService_Request_place_pose(::planning_interfaces::srv::PickPlaceService_Request & msg)
  : msg_(msg)
  {}
  ::planning_interfaces::srv::PickPlaceService_Request place_pose(::planning_interfaces::srv::PickPlaceService_Request::_place_pose_type arg)
  {
    msg_.place_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::planning_interfaces::srv::PickPlaceService_Request msg_;
};

class Init_PickPlaceService_Request_pick_pose
{
public:
  Init_PickPlaceService_Request_pick_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickPlaceService_Request_place_pose pick_pose(::planning_interfaces::srv::PickPlaceService_Request::_pick_pose_type arg)
  {
    msg_.pick_pose = std::move(arg);
    return Init_PickPlaceService_Request_place_pose(msg_);
  }

private:
  ::planning_interfaces::srv::PickPlaceService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::planning_interfaces::srv::PickPlaceService_Request>()
{
  return planning_interfaces::srv::builder::Init_PickPlaceService_Request_pick_pose();
}

}  // namespace planning_interfaces


namespace planning_interfaces
{

namespace srv
{

namespace builder
{

class Init_PickPlaceService_Response_message
{
public:
  explicit Init_PickPlaceService_Response_message(::planning_interfaces::srv::PickPlaceService_Response & msg)
  : msg_(msg)
  {}
  ::planning_interfaces::srv::PickPlaceService_Response message(::planning_interfaces::srv::PickPlaceService_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::planning_interfaces::srv::PickPlaceService_Response msg_;
};

class Init_PickPlaceService_Response_success
{
public:
  Init_PickPlaceService_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickPlaceService_Response_message success(::planning_interfaces::srv::PickPlaceService_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PickPlaceService_Response_message(msg_);
  }

private:
  ::planning_interfaces::srv::PickPlaceService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::planning_interfaces::srv::PickPlaceService_Response>()
{
  return planning_interfaces::srv::builder::Init_PickPlaceService_Response_success();
}

}  // namespace planning_interfaces

#endif  // PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__BUILDER_HPP_
