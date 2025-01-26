// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from task_manager:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/msg/task.hpp"


#ifndef TASK_MANAGER__MSG__DETAIL__TASK__BUILDER_HPP_
#define TASK_MANAGER__MSG__DETAIL__TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "task_manager/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace task_manager
{

namespace msg
{

namespace builder
{

class Init_Task_quantity
{
public:
  explicit Init_Task_quantity(::task_manager::msg::Task & msg)
  : msg_(msg)
  {}
  ::task_manager::msg::Task quantity(::task_manager::msg::Task::_quantity_type arg)
  {
    msg_.quantity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::msg::Task msg_;
};

class Init_Task_item
{
public:
  Init_Task_item()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Task_quantity item(::task_manager::msg::Task::_item_type arg)
  {
    msg_.item = std::move(arg);
    return Init_Task_quantity(msg_);
  }

private:
  ::task_manager::msg::Task msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::msg::Task>()
{
  return task_manager::msg::builder::Init_Task_item();
}

}  // namespace task_manager

#endif  // TASK_MANAGER__MSG__DETAIL__TASK__BUILDER_HPP_
