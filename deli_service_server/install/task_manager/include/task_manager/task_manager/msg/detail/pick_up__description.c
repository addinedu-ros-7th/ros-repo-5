// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from task_manager:msg/PickUP.idl
// generated code does not contain a copyright notice

#include "task_manager/msg/detail/pick_up__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_task_manager
const rosidl_type_hash_t *
task_manager__msg__PickUP__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa0, 0xec, 0x11, 0xb9, 0x73, 0x21, 0x01, 0xd5,
      0x2d, 0x51, 0x75, 0xb7, 0x26, 0x8e, 0x78, 0x4c,
      0xda, 0xa1, 0x9e, 0xe4, 0x3d, 0xc3, 0x79, 0xd6,
      0x12, 0x98, 0x68, 0xc6, 0xca, 0x2d, 0xd9, 0x22,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char task_manager__msg__PickUP__TYPE_NAME[] = "task_manager/msg/PickUP";

// Define type names, field names, and default values
static char task_manager__msg__PickUP__FIELD_NAME__item[] = "item";
static char task_manager__msg__PickUP__FIELD_NAME__quantity[] = "quantity";

static rosidl_runtime_c__type_description__Field task_manager__msg__PickUP__FIELDS[] = {
  {
    {task_manager__msg__PickUP__FIELD_NAME__item, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {task_manager__msg__PickUP__FIELD_NAME__quantity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
task_manager__msg__PickUP__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {task_manager__msg__PickUP__TYPE_NAME, 23, 23},
      {task_manager__msg__PickUP__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string item\n"
  "int32 quantity";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
task_manager__msg__PickUP__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {task_manager__msg__PickUP__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
task_manager__msg__PickUP__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *task_manager__msg__PickUP__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
