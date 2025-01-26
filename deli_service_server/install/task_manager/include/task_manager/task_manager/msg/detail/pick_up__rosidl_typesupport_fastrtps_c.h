// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from task_manager:msg/PickUP.idl
// generated code does not contain a copyright notice
#ifndef TASK_MANAGER__MSG__DETAIL__PICK_UP__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define TASK_MANAGER__MSG__DETAIL__PICK_UP__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "task_manager/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "task_manager/msg/detail/pick_up__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
bool cdr_serialize_task_manager__msg__PickUP(
  const task_manager__msg__PickUP * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
bool cdr_deserialize_task_manager__msg__PickUP(
  eprosima::fastcdr::Cdr &,
  task_manager__msg__PickUP * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
size_t get_serialized_size_task_manager__msg__PickUP(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
size_t max_serialized_size_task_manager__msg__PickUP(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
bool cdr_serialize_key_task_manager__msg__PickUP(
  const task_manager__msg__PickUP * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
size_t get_serialized_size_key_task_manager__msg__PickUP(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
size_t max_serialized_size_key_task_manager__msg__PickUP(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_task_manager
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, task_manager, msg, PickUP)();

#ifdef __cplusplus
}
#endif

#endif  // TASK_MANAGER__MSG__DETAIL__PICK_UP__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
