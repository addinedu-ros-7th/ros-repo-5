// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from task_manager:msg/Task.idl
// generated code does not contain a copyright notice

#include "task_manager/msg/detail/task__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_task_manager
const rosidl_type_hash_t *
task_manager__msg__Task__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x22, 0x15, 0xa5, 0xcc, 0x10, 0x76, 0xe9, 0x22,
      0x3a, 0xca, 0xc0, 0xa7, 0x96, 0x15, 0x8b, 0x27,
      0xe5, 0x73, 0xa0, 0xb0, 0xf1, 0xbd, 0xa5, 0xde,
      0x44, 0x7a, 0x7f, 0x07, 0x30, 0xbd, 0xce, 0x2f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char task_manager__msg__Task__TYPE_NAME[] = "task_manager/msg/Task";

// Define type names, field names, and default values
static char task_manager__msg__Task__FIELD_NAME__item[] = "item";
static char task_manager__msg__Task__FIELD_NAME__quantity[] = "quantity";

static rosidl_runtime_c__type_description__Field task_manager__msg__Task__FIELDS[] = {
  {
    {task_manager__msg__Task__FIELD_NAME__item, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {task_manager__msg__Task__FIELD_NAME__quantity, 8, 8},
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
task_manager__msg__Task__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {task_manager__msg__Task__TYPE_NAME, 21, 21},
      {task_manager__msg__Task__FIELDS, 2, 2},
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
task_manager__msg__Task__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {task_manager__msg__Task__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
task_manager__msg__Task__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *task_manager__msg__Task__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
