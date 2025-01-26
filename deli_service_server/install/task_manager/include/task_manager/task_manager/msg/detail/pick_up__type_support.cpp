// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from task_manager:msg/PickUP.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "task_manager/msg/detail/pick_up__functions.h"
#include "task_manager/msg/detail/pick_up__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace task_manager
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PickUP_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) task_manager::msg::PickUP(_init);
}

void PickUP_fini_function(void * message_memory)
{
  auto typed_message = static_cast<task_manager::msg::PickUP *>(message_memory);
  typed_message->~PickUP();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PickUP_message_member_array[2] = {
  {
    "item",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(task_manager::msg::PickUP, item),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "quantity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(task_manager::msg::PickUP, quantity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PickUP_message_members = {
  "task_manager::msg",  // message namespace
  "PickUP",  // message name
  2,  // number of fields
  sizeof(task_manager::msg::PickUP),
  false,  // has_any_key_member_
  PickUP_message_member_array,  // message members
  PickUP_init_function,  // function to initialize message memory (memory has to be allocated)
  PickUP_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PickUP_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PickUP_message_members,
  get_message_typesupport_handle_function,
  &task_manager__msg__PickUP__get_type_hash,
  &task_manager__msg__PickUP__get_type_description,
  &task_manager__msg__PickUP__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace task_manager


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<task_manager::msg::PickUP>()
{
  return &::task_manager::msg::rosidl_typesupport_introspection_cpp::PickUP_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, task_manager, msg, PickUP)() {
  return &::task_manager::msg::rosidl_typesupport_introspection_cpp::PickUP_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
