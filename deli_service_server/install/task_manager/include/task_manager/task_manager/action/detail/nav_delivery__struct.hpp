// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from task_manager:action/NavDelivery.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/action/nav_delivery.hpp"


#ifndef TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__STRUCT_HPP_
#define TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_Goal __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_Goal __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_Goal_
{
  using Type = NavDelivery_Goal_<ContainerAllocator>;

  explicit NavDelivery_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit NavDelivery_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _stations_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _stations_type stations;

  // setters for named parameter idiom
  Type & set__stations(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->stations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_Goal
    std::shared_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_Goal
    std::shared_ptr<task_manager::action::NavDelivery_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_Goal_ & other) const
  {
    if (this->stations != other.stations) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_Goal_

// alias to use template instance with default allocator
using NavDelivery_Goal =
  task_manager::action::NavDelivery_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager


#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_Result __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_Result __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_Result_
{
  using Type = NavDelivery_Result_<ContainerAllocator>;

  explicit NavDelivery_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0l;
      this->error_msg = "";
    }
  }

  explicit NavDelivery_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_msg(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_code = 0l;
      this->error_msg = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_code_type =
    int32_t;
  _error_code_type error_code;
  using _error_msg_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_msg_type error_msg;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_code(
    const int32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__error_msg(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_msg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_Result
    std::shared_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_Result
    std::shared_ptr<task_manager::action::NavDelivery_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->error_msg != other.error_msg) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_Result_

// alias to use template instance with default allocator
using NavDelivery_Result =
  task_manager::action::NavDelivery_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_Feedback __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_Feedback_
{
  using Type = NavDelivery_Feedback_<ContainerAllocator>;

  explicit NavDelivery_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance_remaining = 0.0f;
    }
  }

  explicit NavDelivery_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance_remaining = 0.0f;
    }
  }

  // field types and members
  using _current_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_pose_type current_pose;
  using _distance_remaining_type =
    float;
  _distance_remaining_type distance_remaining;

  // setters for named parameter idiom
  Type & set__current_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }
  Type & set__distance_remaining(
    const float & _arg)
  {
    this->distance_remaining = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_Feedback
    std::shared_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_Feedback
    std::shared_ptr<task_manager::action::NavDelivery_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_Feedback_ & other) const
  {
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->distance_remaining != other.distance_remaining) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_Feedback_

// alias to use template instance with default allocator
using NavDelivery_Feedback =
  task_manager::action::NavDelivery_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "task_manager/action/detail/nav_delivery__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_SendGoal_Request __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_SendGoal_Request_
{
  using Type = NavDelivery_SendGoal_Request_<ContainerAllocator>;

  explicit NavDelivery_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit NavDelivery_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    task_manager::action::NavDelivery_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const task_manager::action::NavDelivery_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_SendGoal_Request
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_SendGoal_Request
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_SendGoal_Request_

// alias to use template instance with default allocator
using NavDelivery_SendGoal_Request =
  task_manager::action::NavDelivery_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_SendGoal_Response __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_SendGoal_Response_
{
  using Type = NavDelivery_SendGoal_Response_<ContainerAllocator>;

  explicit NavDelivery_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit NavDelivery_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_SendGoal_Response
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_SendGoal_Response
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_SendGoal_Response_

// alias to use template instance with default allocator
using NavDelivery_SendGoal_Response =
  task_manager::action::NavDelivery_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_SendGoal_Event __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_SendGoal_Event_
{
  using Type = NavDelivery_SendGoal_Event_<ContainerAllocator>;

  explicit NavDelivery_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit NavDelivery_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_SendGoal_Event
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_SendGoal_Event
    std::shared_ptr<task_manager::action::NavDelivery_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_SendGoal_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_SendGoal_Event_

// alias to use template instance with default allocator
using NavDelivery_SendGoal_Event =
  task_manager::action::NavDelivery_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager

namespace task_manager
{

namespace action
{

struct NavDelivery_SendGoal
{
  using Request = task_manager::action::NavDelivery_SendGoal_Request;
  using Response = task_manager::action::NavDelivery_SendGoal_Response;
  using Event = task_manager::action::NavDelivery_SendGoal_Event;
};

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_GetResult_Request __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_GetResult_Request_
{
  using Type = NavDelivery_GetResult_Request_<ContainerAllocator>;

  explicit NavDelivery_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit NavDelivery_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_GetResult_Request
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_GetResult_Request
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_GetResult_Request_

// alias to use template instance with default allocator
using NavDelivery_GetResult_Request =
  task_manager::action::NavDelivery_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'result'
// already included above
// #include "task_manager/action/detail/nav_delivery__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_GetResult_Response __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_GetResult_Response_
{
  using Type = NavDelivery_GetResult_Response_<ContainerAllocator>;

  explicit NavDelivery_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit NavDelivery_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    task_manager::action::NavDelivery_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const task_manager::action::NavDelivery_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_GetResult_Response
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_GetResult_Response
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_GetResult_Response_

// alias to use template instance with default allocator
using NavDelivery_GetResult_Response =
  task_manager::action::NavDelivery_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_GetResult_Event __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_GetResult_Event_
{
  using Type = NavDelivery_GetResult_Event_<ContainerAllocator>;

  explicit NavDelivery_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit NavDelivery_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<task_manager::action::NavDelivery_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_GetResult_Event
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_GetResult_Event
    std::shared_ptr<task_manager::action::NavDelivery_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_GetResult_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_GetResult_Event_

// alias to use template instance with default allocator
using NavDelivery_GetResult_Event =
  task_manager::action::NavDelivery_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager

namespace task_manager
{

namespace action
{

struct NavDelivery_GetResult
{
  using Request = task_manager::action::NavDelivery_GetResult_Request;
  using Response = task_manager::action::NavDelivery_GetResult_Response;
  using Event = task_manager::action::NavDelivery_GetResult_Event;
};

}  // namespace action

}  // namespace task_manager


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "task_manager/action/detail/nav_delivery__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_manager__action__NavDelivery_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__action__NavDelivery_FeedbackMessage __declspec(deprecated)
#endif

namespace task_manager
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavDelivery_FeedbackMessage_
{
  using Type = NavDelivery_FeedbackMessage_<ContainerAllocator>;

  explicit NavDelivery_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit NavDelivery_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    task_manager::action::NavDelivery_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const task_manager::action::NavDelivery_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__action__NavDelivery_FeedbackMessage
    std::shared_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__action__NavDelivery_FeedbackMessage
    std::shared_ptr<task_manager::action::NavDelivery_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDelivery_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDelivery_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDelivery_FeedbackMessage_

// alias to use template instance with default allocator
using NavDelivery_FeedbackMessage =
  task_manager::action::NavDelivery_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace task_manager

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace task_manager
{

namespace action
{

struct NavDelivery
{
  /// The goal message defined in the action definition.
  using Goal = task_manager::action::NavDelivery_Goal;
  /// The result message defined in the action definition.
  using Result = task_manager::action::NavDelivery_Result;
  /// The feedback message defined in the action definition.
  using Feedback = task_manager::action::NavDelivery_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = task_manager::action::NavDelivery_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = task_manager::action::NavDelivery_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = task_manager::action::NavDelivery_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct NavDelivery NavDelivery;

}  // namespace action

}  // namespace task_manager

#endif  // TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__STRUCT_HPP_
