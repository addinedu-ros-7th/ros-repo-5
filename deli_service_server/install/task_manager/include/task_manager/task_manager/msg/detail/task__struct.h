// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from task_manager:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/msg/task.h"


#ifndef TASK_MANAGER__MSG__DETAIL__TASK__STRUCT_H_
#define TASK_MANAGER__MSG__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'item'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Task in the package task_manager.
typedef struct task_manager__msg__Task
{
  rosidl_runtime_c__String item;
  int32_t quantity;
} task_manager__msg__Task;

// Struct for a sequence of task_manager__msg__Task.
typedef struct task_manager__msg__Task__Sequence
{
  task_manager__msg__Task * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__msg__Task__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TASK_MANAGER__MSG__DETAIL__TASK__STRUCT_H_
