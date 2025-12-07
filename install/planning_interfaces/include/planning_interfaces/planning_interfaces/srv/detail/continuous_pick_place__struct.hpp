// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from planning_interfaces:srv/ContinuousPickPlace.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__STRUCT_HPP_
#define PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pick_scan_pose'
// Member 'place_scan_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Request __attribute__((deprecated))
#else
# define DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Request __declspec(deprecated)
#endif

namespace planning_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ContinuousPickPlace_Request_
{
  using Type = ContinuousPickPlace_Request_<ContainerAllocator>;

  explicit ContinuousPickPlace_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pick_scan_pose(_init),
    place_scan_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pick_distance = 0.0f;
      this->retreat_distance = 0.0f;
      this->place_distance = 0.0f;
    }
  }

  explicit ContinuousPickPlace_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pick_scan_pose(_alloc, _init),
    place_scan_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pick_distance = 0.0f;
      this->retreat_distance = 0.0f;
      this->place_distance = 0.0f;
    }
  }

  // field types and members
  using _pick_scan_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pick_scan_pose_type pick_scan_pose;
  using _place_scan_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _place_scan_pose_type place_scan_pose;
  using _pick_distance_type =
    float;
  _pick_distance_type pick_distance;
  using _retreat_distance_type =
    float;
  _retreat_distance_type retreat_distance;
  using _place_distance_type =
    float;
  _place_distance_type place_distance;

  // setters for named parameter idiom
  Type & set__pick_scan_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pick_scan_pose = _arg;
    return *this;
  }
  Type & set__place_scan_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->place_scan_pose = _arg;
    return *this;
  }
  Type & set__pick_distance(
    const float & _arg)
  {
    this->pick_distance = _arg;
    return *this;
  }
  Type & set__retreat_distance(
    const float & _arg)
  {
    this->retreat_distance = _arg;
    return *this;
  }
  Type & set__place_distance(
    const float & _arg)
  {
    this->place_distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Request
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Request
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContinuousPickPlace_Request_ & other) const
  {
    if (this->pick_scan_pose != other.pick_scan_pose) {
      return false;
    }
    if (this->place_scan_pose != other.place_scan_pose) {
      return false;
    }
    if (this->pick_distance != other.pick_distance) {
      return false;
    }
    if (this->retreat_distance != other.retreat_distance) {
      return false;
    }
    if (this->place_distance != other.place_distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContinuousPickPlace_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContinuousPickPlace_Request_

// alias to use template instance with default allocator
using ContinuousPickPlace_Request =
  planning_interfaces::srv::ContinuousPickPlace_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace planning_interfaces


#ifndef _WIN32
# define DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Response __attribute__((deprecated))
#else
# define DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Response __declspec(deprecated)
#endif

namespace planning_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ContinuousPickPlace_Response_
{
  using Type = ContinuousPickPlace_Response_<ContainerAllocator>;

  explicit ContinuousPickPlace_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->slides_picked = 0l;
    }
  }

  explicit ContinuousPickPlace_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->slides_picked = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _slides_picked_type =
    int32_t;
  _slides_picked_type slides_picked;

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
  Type & set__slides_picked(
    const int32_t & _arg)
  {
    this->slides_picked = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Response
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__planning_interfaces__srv__ContinuousPickPlace_Response
    std::shared_ptr<planning_interfaces::srv::ContinuousPickPlace_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContinuousPickPlace_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->slides_picked != other.slides_picked) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContinuousPickPlace_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContinuousPickPlace_Response_

// alias to use template instance with default allocator
using ContinuousPickPlace_Response =
  planning_interfaces::srv::ContinuousPickPlace_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace planning_interfaces

namespace planning_interfaces
{

namespace srv
{

struct ContinuousPickPlace
{
  using Request = planning_interfaces::srv::ContinuousPickPlace_Request;
  using Response = planning_interfaces::srv::ContinuousPickPlace_Response;
};

}  // namespace srv

}  // namespace planning_interfaces

#endif  // PLANNING_INTERFACES__SRV__DETAIL__CONTINUOUS_PICK_PLACE__STRUCT_HPP_
