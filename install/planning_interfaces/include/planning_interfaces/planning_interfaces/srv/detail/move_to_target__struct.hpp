// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from planning_interfaces:srv/MoveToTarget.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__STRUCT_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__planning_interfaces__srv__MoveToTarget_Request __attribute__((deprecated))
#else
# define DEPRECATED__planning_interfaces__srv__MoveToTarget_Request __declspec(deprecated)
#endif

namespace planning_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToTarget_Request_
{
  using Type = MoveToTarget_Request_<ContainerAllocator>;

  explicit MoveToTarget_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_init)
  {
    (void)_init;
  }

  explicit MoveToTarget_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _target_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _target_pose_type target_pose;

  // setters for named parameter idiom
  Type & set__target_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__planning_interfaces__srv__MoveToTarget_Request
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__planning_interfaces__srv__MoveToTarget_Request
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToTarget_Request_ & other) const
  {
    if (this->target_pose != other.target_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToTarget_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToTarget_Request_

// alias to use template instance with default allocator
using MoveToTarget_Request =
  planning_interfaces::srv::MoveToTarget_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace planning_interfaces


#ifndef _WIN32
# define DEPRECATED__planning_interfaces__srv__MoveToTarget_Response __attribute__((deprecated))
#else
# define DEPRECATED__planning_interfaces__srv__MoveToTarget_Response __declspec(deprecated)
#endif

namespace planning_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToTarget_Response_
{
  using Type = MoveToTarget_Response_<ContainerAllocator>;

  explicit MoveToTarget_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit MoveToTarget_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__planning_interfaces__srv__MoveToTarget_Response
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__planning_interfaces__srv__MoveToTarget_Response
    std::shared_ptr<planning_interfaces::srv::MoveToTarget_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToTarget_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToTarget_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToTarget_Response_

// alias to use template instance with default allocator
using MoveToTarget_Response =
  planning_interfaces::srv::MoveToTarget_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace planning_interfaces

namespace planning_interfaces
{

namespace srv
{

struct MoveToTarget
{
  using Request = planning_interfaces::srv::MoveToTarget_Request;
  using Response = planning_interfaces::srv::MoveToTarget_Response;
};

}  // namespace srv

}  // namespace planning_interfaces

#endif  // PLANNING_INTERFACES__SRV__DETAIL__MOVE_TO_TARGET__STRUCT_HPP_
