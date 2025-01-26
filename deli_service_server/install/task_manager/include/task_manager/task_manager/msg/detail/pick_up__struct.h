// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from task_manager:msg/PickUP.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/msg/pick_up.h"


#ifndef TASK_MANAGER__MSG__DETAIL__PICK_UP__STRUCT_H_
#define TASK_MANAGER__MSG__DETAIL__PICK_UP__STRUCT_H_

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

/// Struct defined in msg/PickUP in the package task_manager.
typedef struct task_manager__msg__PickUP
{
  rosidl_runtime_c__String item;
  int32_t quantity;
} task_manager__msg__PickUP;

// Struct for a sequence of task_manager__msg__PickUP.
typedef struct task_manager__msg__PickUP__Sequence
{
  task_manager__msg__PickUP * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__msg__PickUP__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TASK_MANAGER__MSG__DETAIL__PICK_UP__STRUCT_H_
