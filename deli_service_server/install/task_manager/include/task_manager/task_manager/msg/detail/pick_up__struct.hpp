// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from task_manager:msg/PickUP.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/msg/pick_up.hpp"


#ifndef TASK_MANAGER__MSG__DETAIL__PICK_UP__STRUCT_HPP_
#define TASK_MANAGER__MSG__DETAIL__PICK_UP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__task_manager__msg__PickUP __attribute__((deprecated))
#else
# define DEPRECATED__task_manager__msg__PickUP __declspec(deprecated)
#endif

namespace task_manager
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PickUP_
{
  using Type = PickUP_<ContainerAllocator>;

  explicit PickUP_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->item = "";
      this->quantity = 0l;
    }
  }

  explicit PickUP_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : item(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->item = "";
      this->quantity = 0l;
    }
  }

  // field types and members
  using _item_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _item_type item;
  using _quantity_type =
    int32_t;
  _quantity_type quantity;

  // setters for named parameter idiom
  Type & set__item(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->item = _arg;
    return *this;
  }
  Type & set__quantity(
    const int32_t & _arg)
  {
    this->quantity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_manager::msg::PickUP_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_manager::msg::PickUP_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_manager::msg::PickUP_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_manager::msg::PickUP_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_manager::msg::PickUP_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_manager::msg::PickUP_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_manager::msg::PickUP_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_manager::msg::PickUP_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_manager::msg::PickUP_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_manager::msg::PickUP_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_manager__msg__PickUP
    std::shared_ptr<task_manager::msg::PickUP_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_manager__msg__PickUP
    std::shared_ptr<task_manager::msg::PickUP_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickUP_ & other) const
  {
    if (this->item != other.item) {
      return false;
    }
    if (this->quantity != other.quantity) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickUP_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickUP_

// alias to use template instance with default allocator
using PickUP =
  task_manager::msg::PickUP_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace task_manager

#endif  // TASK_MANAGER__MSG__DETAIL__PICK_UP__STRUCT_HPP_
