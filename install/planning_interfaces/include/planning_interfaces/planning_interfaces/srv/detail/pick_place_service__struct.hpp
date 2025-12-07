// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from planning_interfaces:srv/PickPlaceService.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__STRUCT_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pick_pose'
// Member 'place_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__planning_interfaces__srv__PickPlaceService_Request __attribute__((deprecated))
#else
# define DEPRECATED__planning_interfaces__srv__PickPlaceService_Request __declspec(deprecated)
#endif

namespace planning_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PickPlaceService_Request_
{
  using Type = PickPlaceService_Request_<ContainerAllocator>;

  explicit PickPlaceService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pick_pose(_init),
    place_pose(_init)
  {
    (void)_init;
  }

  explicit PickPlaceService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pick_pose(_alloc, _init),
    place_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pick_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pick_pose_type pick_pose;
  using _place_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _place_pose_type place_pose;

  // setters for named parameter idiom
  Type & set__pick_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pick_pose = _arg;
    return *this;
  }
  Type & set__place_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->place_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__planning_interfaces__srv__PickPlaceService_Request
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__planning_interfaces__srv__PickPlaceService_Request
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPlaceService_Request_ & other) const
  {
    if (this->pick_pose != other.pick_pose) {
      return false;
    }
    if (this->place_pose != other.place_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPlaceService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPlaceService_Request_

// alias to use template instance with default allocator
using PickPlaceService_Request =
  planning_interfaces::srv::PickPlaceService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace planning_interfaces


#ifndef _WIN32
# define DEPRECATED__planning_interfaces__srv__PickPlaceService_Response __attribute__((deprecated))
#else
# define DEPRECATED__planning_interfaces__srv__PickPlaceService_Response __declspec(deprecated)
#endif

namespace planning_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PickPlaceService_Response_
{
  using Type = PickPlaceService_Response_<ContainerAllocator>;

  explicit PickPlaceService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PickPlaceService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__planning_interfaces__srv__PickPlaceService_Response
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__planning_interfaces__srv__PickPlaceService_Response
    std::shared_ptr<planning_interfaces::srv::PickPlaceService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPlaceService_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPlaceService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPlaceService_Response_

// alias to use template instance with default allocator
using PickPlaceService_Response =
  planning_interfaces::srv::PickPlaceService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace planning_interfaces

namespace planning_interfaces
{

namespace srv
{

struct PickPlaceService
{
  using Request = planning_interfaces::srv::PickPlaceService_Request;
  using Response = planning_interfaces::srv::PickPlaceService_Response;
};

}  // namespace srv

}  // namespace planning_interfaces

#endif  // PLANNING_INTERFACES__SRV__DETAIL__PICK_PLACE_SERVICE__STRUCT_HPP_
