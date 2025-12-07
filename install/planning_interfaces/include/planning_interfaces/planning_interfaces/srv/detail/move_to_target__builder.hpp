// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from planning_interfaces:srv/MoveToTarget.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__BUILDER_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "planning_interfaces/srv/detail/move_to_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace planning_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveToTarget_Request_target_pose
{
public:
  Init_MoveToTarget_Request_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::planning_interfaces::srv::MoveToTarget_Request target_pose(::planning_interfaces::srv::MoveToTarget_Request::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::planning_interfaces::srv::MoveToTarget_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::planning_interfaces::srv::MoveToTarget_Request>()
{
  return planning_interfaces::srv::builder::Init_MoveToTarget_Request_target_pose();
}

}  // namespace planning_interfaces


namespace planning_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveToTarget_Response_message
{
public:
  explicit Init_MoveToTarget_Response_message(::planning_interfaces::srv::MoveToTarget_Response & msg)
  : msg_(msg)
  {}
  ::planning_interfaces::srv::MoveToTarget_Response message(::planning_interfaces::srv::MoveToTarget_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::planning_interfaces::srv::MoveToTarget_Response msg_;
};

class Init_MoveToTarget_Response_success
{
public:
  Init_MoveToTarget_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToTarget_Response_message success(::planning_interfaces::srv::MoveToTarget_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveToTarget_Response_message(msg_);
  }

private:
  ::planning_interfaces::srv::MoveToTarget_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::planning_interfaces::srv::MoveToTarget_Response>()
{
  return planning_interfaces::srv::builder::Init_MoveToTarget_Response_success();
}

}  // namespace planning_interfaces

#endif  // PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__BUILDER_HPP_
