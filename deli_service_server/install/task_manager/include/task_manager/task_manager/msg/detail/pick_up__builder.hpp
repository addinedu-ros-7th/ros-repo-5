// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from task_manager:msg/PickUP.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/msg/pick_up.hpp"


#ifndef TASK_MANAGER__MSG__DETAIL__PICK_UP__BUILDER_HPP_
#define TASK_MANAGER__MSG__DETAIL__PICK_UP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "task_manager/msg/detail/pick_up__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace task_manager
{

namespace msg
{

namespace builder
{

class Init_PickUP_quantity
{
public:
  explicit Init_PickUP_quantity(::task_manager::msg::PickUP & msg)
  : msg_(msg)
  {}
  ::task_manager::msg::PickUP quantity(::task_manager::msg::PickUP::_quantity_type arg)
  {
    msg_.quantity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::msg::PickUP msg_;
};

class Init_PickUP_item
{
public:
  Init_PickUP_item()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickUP_quantity item(::task_manager::msg::PickUP::_item_type arg)
  {
    msg_.item = std::move(arg);
    return Init_PickUP_quantity(msg_);
  }

private:
  ::task_manager::msg::PickUP msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::msg::PickUP>()
{
  return task_manager::msg::builder::Init_PickUP_item();
}

}  // namespace task_manager

#endif  // TASK_MANAGER__MSG__DETAIL__PICK_UP__BUILDER_HPP_
