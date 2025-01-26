// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from task_manager:action/NavDelivery.idl
// generated code does not contain a copyright notice
#include "task_manager/action/detail/nav_delivery__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stations`
#include "rosidl_runtime_c/string_functions.h"

bool
task_manager__action__NavDelivery_Goal__init(task_manager__action__NavDelivery_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // stations
  if (!rosidl_runtime_c__String__Sequence__init(&msg->stations, 0)) {
    task_manager__action__NavDelivery_Goal__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_Goal__fini(task_manager__action__NavDelivery_Goal * msg)
{
  if (!msg) {
    return;
  }
  // stations
  rosidl_runtime_c__String__Sequence__fini(&msg->stations);
}

bool
task_manager__action__NavDelivery_Goal__are_equal(const task_manager__action__NavDelivery_Goal * lhs, const task_manager__action__NavDelivery_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stations
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->stations), &(rhs->stations)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_Goal__copy(
  const task_manager__action__NavDelivery_Goal * input,
  task_manager__action__NavDelivery_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // stations
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->stations), &(output->stations)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_Goal *
task_manager__action__NavDelivery_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Goal * msg = (task_manager__action__NavDelivery_Goal *)allocator.allocate(sizeof(task_manager__action__NavDelivery_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_Goal));
  bool success = task_manager__action__NavDelivery_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_Goal__destroy(task_manager__action__NavDelivery_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_Goal__Sequence__init(task_manager__action__NavDelivery_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Goal * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_Goal *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_Goal__Sequence__fini(task_manager__action__NavDelivery_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_Goal__Sequence *
task_manager__action__NavDelivery_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Goal__Sequence * array = (task_manager__action__NavDelivery_Goal__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_Goal__Sequence__destroy(task_manager__action__NavDelivery_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_Goal__Sequence__are_equal(const task_manager__action__NavDelivery_Goal__Sequence * lhs, const task_manager__action__NavDelivery_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_Goal__Sequence__copy(
  const task_manager__action__NavDelivery_Goal__Sequence * input,
  task_manager__action__NavDelivery_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_Goal * data =
      (task_manager__action__NavDelivery_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `error_msg`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
task_manager__action__NavDelivery_Result__init(task_manager__action__NavDelivery_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // error_code
  // error_msg
  if (!rosidl_runtime_c__String__init(&msg->error_msg)) {
    task_manager__action__NavDelivery_Result__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_Result__fini(task_manager__action__NavDelivery_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // error_code
  // error_msg
  rosidl_runtime_c__String__fini(&msg->error_msg);
}

bool
task_manager__action__NavDelivery_Result__are_equal(const task_manager__action__NavDelivery_Result * lhs, const task_manager__action__NavDelivery_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // error_msg
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_msg), &(rhs->error_msg)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_Result__copy(
  const task_manager__action__NavDelivery_Result * input,
  task_manager__action__NavDelivery_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // error_code
  output->error_code = input->error_code;
  // error_msg
  if (!rosidl_runtime_c__String__copy(
      &(input->error_msg), &(output->error_msg)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_Result *
task_manager__action__NavDelivery_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Result * msg = (task_manager__action__NavDelivery_Result *)allocator.allocate(sizeof(task_manager__action__NavDelivery_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_Result));
  bool success = task_manager__action__NavDelivery_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_Result__destroy(task_manager__action__NavDelivery_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_Result__Sequence__init(task_manager__action__NavDelivery_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Result * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_Result *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_Result__Sequence__fini(task_manager__action__NavDelivery_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_Result__Sequence *
task_manager__action__NavDelivery_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Result__Sequence * array = (task_manager__action__NavDelivery_Result__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_Result__Sequence__destroy(task_manager__action__NavDelivery_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_Result__Sequence__are_equal(const task_manager__action__NavDelivery_Result__Sequence * lhs, const task_manager__action__NavDelivery_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_Result__Sequence__copy(
  const task_manager__action__NavDelivery_Result__Sequence * input,
  task_manager__action__NavDelivery_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_Result * data =
      (task_manager__action__NavDelivery_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `current_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
task_manager__action__NavDelivery_Feedback__init(task_manager__action__NavDelivery_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    task_manager__action__NavDelivery_Feedback__fini(msg);
    return false;
  }
  // distance_remaining
  return true;
}

void
task_manager__action__NavDelivery_Feedback__fini(task_manager__action__NavDelivery_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
  // distance_remaining
}

bool
task_manager__action__NavDelivery_Feedback__are_equal(const task_manager__action__NavDelivery_Feedback * lhs, const task_manager__action__NavDelivery_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  // distance_remaining
  if (lhs->distance_remaining != rhs->distance_remaining) {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_Feedback__copy(
  const task_manager__action__NavDelivery_Feedback * input,
  task_manager__action__NavDelivery_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  // distance_remaining
  output->distance_remaining = input->distance_remaining;
  return true;
}

task_manager__action__NavDelivery_Feedback *
task_manager__action__NavDelivery_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Feedback * msg = (task_manager__action__NavDelivery_Feedback *)allocator.allocate(sizeof(task_manager__action__NavDelivery_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_Feedback));
  bool success = task_manager__action__NavDelivery_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_Feedback__destroy(task_manager__action__NavDelivery_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_Feedback__Sequence__init(task_manager__action__NavDelivery_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Feedback * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_Feedback *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_Feedback__Sequence__fini(task_manager__action__NavDelivery_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_Feedback__Sequence *
task_manager__action__NavDelivery_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_Feedback__Sequence * array = (task_manager__action__NavDelivery_Feedback__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_Feedback__Sequence__destroy(task_manager__action__NavDelivery_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_Feedback__Sequence__are_equal(const task_manager__action__NavDelivery_Feedback__Sequence * lhs, const task_manager__action__NavDelivery_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_Feedback__Sequence__copy(
  const task_manager__action__NavDelivery_Feedback__Sequence * input,
  task_manager__action__NavDelivery_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_Feedback * data =
      (task_manager__action__NavDelivery_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "task_manager/action/detail/nav_delivery__functions.h"

bool
task_manager__action__NavDelivery_SendGoal_Request__init(task_manager__action__NavDelivery_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    task_manager__action__NavDelivery_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!task_manager__action__NavDelivery_Goal__init(&msg->goal)) {
    task_manager__action__NavDelivery_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_SendGoal_Request__fini(task_manager__action__NavDelivery_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  task_manager__action__NavDelivery_Goal__fini(&msg->goal);
}

bool
task_manager__action__NavDelivery_SendGoal_Request__are_equal(const task_manager__action__NavDelivery_SendGoal_Request * lhs, const task_manager__action__NavDelivery_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!task_manager__action__NavDelivery_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_SendGoal_Request__copy(
  const task_manager__action__NavDelivery_SendGoal_Request * input,
  task_manager__action__NavDelivery_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!task_manager__action__NavDelivery_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_SendGoal_Request *
task_manager__action__NavDelivery_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Request * msg = (task_manager__action__NavDelivery_SendGoal_Request *)allocator.allocate(sizeof(task_manager__action__NavDelivery_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_SendGoal_Request));
  bool success = task_manager__action__NavDelivery_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_SendGoal_Request__destroy(task_manager__action__NavDelivery_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_SendGoal_Request__Sequence__init(task_manager__action__NavDelivery_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Request * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_SendGoal_Request *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_SendGoal_Request__Sequence__fini(task_manager__action__NavDelivery_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_SendGoal_Request__Sequence *
task_manager__action__NavDelivery_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Request__Sequence * array = (task_manager__action__NavDelivery_SendGoal_Request__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_SendGoal_Request__Sequence__destroy(task_manager__action__NavDelivery_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_SendGoal_Request__Sequence__are_equal(const task_manager__action__NavDelivery_SendGoal_Request__Sequence * lhs, const task_manager__action__NavDelivery_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_SendGoal_Request__Sequence__copy(
  const task_manager__action__NavDelivery_SendGoal_Request__Sequence * input,
  task_manager__action__NavDelivery_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_SendGoal_Request * data =
      (task_manager__action__NavDelivery_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
task_manager__action__NavDelivery_SendGoal_Response__init(task_manager__action__NavDelivery_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    task_manager__action__NavDelivery_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_SendGoal_Response__fini(task_manager__action__NavDelivery_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
task_manager__action__NavDelivery_SendGoal_Response__are_equal(const task_manager__action__NavDelivery_SendGoal_Response * lhs, const task_manager__action__NavDelivery_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_SendGoal_Response__copy(
  const task_manager__action__NavDelivery_SendGoal_Response * input,
  task_manager__action__NavDelivery_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_SendGoal_Response *
task_manager__action__NavDelivery_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Response * msg = (task_manager__action__NavDelivery_SendGoal_Response *)allocator.allocate(sizeof(task_manager__action__NavDelivery_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_SendGoal_Response));
  bool success = task_manager__action__NavDelivery_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_SendGoal_Response__destroy(task_manager__action__NavDelivery_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_SendGoal_Response__Sequence__init(task_manager__action__NavDelivery_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Response * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_SendGoal_Response *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_SendGoal_Response__Sequence__fini(task_manager__action__NavDelivery_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_SendGoal_Response__Sequence *
task_manager__action__NavDelivery_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Response__Sequence * array = (task_manager__action__NavDelivery_SendGoal_Response__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_SendGoal_Response__Sequence__destroy(task_manager__action__NavDelivery_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_SendGoal_Response__Sequence__are_equal(const task_manager__action__NavDelivery_SendGoal_Response__Sequence * lhs, const task_manager__action__NavDelivery_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_SendGoal_Response__Sequence__copy(
  const task_manager__action__NavDelivery_SendGoal_Response__Sequence * input,
  task_manager__action__NavDelivery_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_SendGoal_Response * data =
      (task_manager__action__NavDelivery_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "task_manager/action/detail/nav_delivery__functions.h"

bool
task_manager__action__NavDelivery_SendGoal_Event__init(task_manager__action__NavDelivery_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    task_manager__action__NavDelivery_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!task_manager__action__NavDelivery_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    task_manager__action__NavDelivery_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!task_manager__action__NavDelivery_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    task_manager__action__NavDelivery_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_SendGoal_Event__fini(task_manager__action__NavDelivery_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  task_manager__action__NavDelivery_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  task_manager__action__NavDelivery_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
task_manager__action__NavDelivery_SendGoal_Event__are_equal(const task_manager__action__NavDelivery_SendGoal_Event * lhs, const task_manager__action__NavDelivery_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!task_manager__action__NavDelivery_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!task_manager__action__NavDelivery_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_SendGoal_Event__copy(
  const task_manager__action__NavDelivery_SendGoal_Event * input,
  task_manager__action__NavDelivery_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!task_manager__action__NavDelivery_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!task_manager__action__NavDelivery_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_SendGoal_Event *
task_manager__action__NavDelivery_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Event * msg = (task_manager__action__NavDelivery_SendGoal_Event *)allocator.allocate(sizeof(task_manager__action__NavDelivery_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_SendGoal_Event));
  bool success = task_manager__action__NavDelivery_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_SendGoal_Event__destroy(task_manager__action__NavDelivery_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_SendGoal_Event__Sequence__init(task_manager__action__NavDelivery_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Event * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_SendGoal_Event *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_SendGoal_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_SendGoal_Event__Sequence__fini(task_manager__action__NavDelivery_SendGoal_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_SendGoal_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_SendGoal_Event__Sequence *
task_manager__action__NavDelivery_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_SendGoal_Event__Sequence * array = (task_manager__action__NavDelivery_SendGoal_Event__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_SendGoal_Event__Sequence__destroy(task_manager__action__NavDelivery_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_SendGoal_Event__Sequence__are_equal(const task_manager__action__NavDelivery_SendGoal_Event__Sequence * lhs, const task_manager__action__NavDelivery_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_SendGoal_Event__Sequence__copy(
  const task_manager__action__NavDelivery_SendGoal_Event__Sequence * input,
  task_manager__action__NavDelivery_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_SendGoal_Event * data =
      (task_manager__action__NavDelivery_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_SendGoal_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
task_manager__action__NavDelivery_GetResult_Request__init(task_manager__action__NavDelivery_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    task_manager__action__NavDelivery_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_GetResult_Request__fini(task_manager__action__NavDelivery_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
task_manager__action__NavDelivery_GetResult_Request__are_equal(const task_manager__action__NavDelivery_GetResult_Request * lhs, const task_manager__action__NavDelivery_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_GetResult_Request__copy(
  const task_manager__action__NavDelivery_GetResult_Request * input,
  task_manager__action__NavDelivery_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_GetResult_Request *
task_manager__action__NavDelivery_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Request * msg = (task_manager__action__NavDelivery_GetResult_Request *)allocator.allocate(sizeof(task_manager__action__NavDelivery_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_GetResult_Request));
  bool success = task_manager__action__NavDelivery_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_GetResult_Request__destroy(task_manager__action__NavDelivery_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_GetResult_Request__Sequence__init(task_manager__action__NavDelivery_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Request * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_GetResult_Request *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_GetResult_Request__Sequence__fini(task_manager__action__NavDelivery_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_GetResult_Request__Sequence *
task_manager__action__NavDelivery_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Request__Sequence * array = (task_manager__action__NavDelivery_GetResult_Request__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_GetResult_Request__Sequence__destroy(task_manager__action__NavDelivery_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_GetResult_Request__Sequence__are_equal(const task_manager__action__NavDelivery_GetResult_Request__Sequence * lhs, const task_manager__action__NavDelivery_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_GetResult_Request__Sequence__copy(
  const task_manager__action__NavDelivery_GetResult_Request__Sequence * input,
  task_manager__action__NavDelivery_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_GetResult_Request * data =
      (task_manager__action__NavDelivery_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "task_manager/action/detail/nav_delivery__functions.h"

bool
task_manager__action__NavDelivery_GetResult_Response__init(task_manager__action__NavDelivery_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!task_manager__action__NavDelivery_Result__init(&msg->result)) {
    task_manager__action__NavDelivery_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_GetResult_Response__fini(task_manager__action__NavDelivery_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  task_manager__action__NavDelivery_Result__fini(&msg->result);
}

bool
task_manager__action__NavDelivery_GetResult_Response__are_equal(const task_manager__action__NavDelivery_GetResult_Response * lhs, const task_manager__action__NavDelivery_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!task_manager__action__NavDelivery_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_GetResult_Response__copy(
  const task_manager__action__NavDelivery_GetResult_Response * input,
  task_manager__action__NavDelivery_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!task_manager__action__NavDelivery_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_GetResult_Response *
task_manager__action__NavDelivery_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Response * msg = (task_manager__action__NavDelivery_GetResult_Response *)allocator.allocate(sizeof(task_manager__action__NavDelivery_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_GetResult_Response));
  bool success = task_manager__action__NavDelivery_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_GetResult_Response__destroy(task_manager__action__NavDelivery_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_GetResult_Response__Sequence__init(task_manager__action__NavDelivery_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Response * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_GetResult_Response *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_GetResult_Response__Sequence__fini(task_manager__action__NavDelivery_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_GetResult_Response__Sequence *
task_manager__action__NavDelivery_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Response__Sequence * array = (task_manager__action__NavDelivery_GetResult_Response__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_GetResult_Response__Sequence__destroy(task_manager__action__NavDelivery_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_GetResult_Response__Sequence__are_equal(const task_manager__action__NavDelivery_GetResult_Response__Sequence * lhs, const task_manager__action__NavDelivery_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_GetResult_Response__Sequence__copy(
  const task_manager__action__NavDelivery_GetResult_Response__Sequence * input,
  task_manager__action__NavDelivery_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_GetResult_Response * data =
      (task_manager__action__NavDelivery_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "task_manager/action/detail/nav_delivery__functions.h"

bool
task_manager__action__NavDelivery_GetResult_Event__init(task_manager__action__NavDelivery_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    task_manager__action__NavDelivery_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!task_manager__action__NavDelivery_GetResult_Request__Sequence__init(&msg->request, 0)) {
    task_manager__action__NavDelivery_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!task_manager__action__NavDelivery_GetResult_Response__Sequence__init(&msg->response, 0)) {
    task_manager__action__NavDelivery_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_GetResult_Event__fini(task_manager__action__NavDelivery_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  task_manager__action__NavDelivery_GetResult_Request__Sequence__fini(&msg->request);
  // response
  task_manager__action__NavDelivery_GetResult_Response__Sequence__fini(&msg->response);
}

bool
task_manager__action__NavDelivery_GetResult_Event__are_equal(const task_manager__action__NavDelivery_GetResult_Event * lhs, const task_manager__action__NavDelivery_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!task_manager__action__NavDelivery_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!task_manager__action__NavDelivery_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_GetResult_Event__copy(
  const task_manager__action__NavDelivery_GetResult_Event * input,
  task_manager__action__NavDelivery_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!task_manager__action__NavDelivery_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!task_manager__action__NavDelivery_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_GetResult_Event *
task_manager__action__NavDelivery_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Event * msg = (task_manager__action__NavDelivery_GetResult_Event *)allocator.allocate(sizeof(task_manager__action__NavDelivery_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_GetResult_Event));
  bool success = task_manager__action__NavDelivery_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_GetResult_Event__destroy(task_manager__action__NavDelivery_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_GetResult_Event__Sequence__init(task_manager__action__NavDelivery_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Event * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_GetResult_Event *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_GetResult_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_GetResult_Event__Sequence__fini(task_manager__action__NavDelivery_GetResult_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_GetResult_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_GetResult_Event__Sequence *
task_manager__action__NavDelivery_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_GetResult_Event__Sequence * array = (task_manager__action__NavDelivery_GetResult_Event__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_GetResult_Event__Sequence__destroy(task_manager__action__NavDelivery_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_GetResult_Event__Sequence__are_equal(const task_manager__action__NavDelivery_GetResult_Event__Sequence * lhs, const task_manager__action__NavDelivery_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_GetResult_Event__Sequence__copy(
  const task_manager__action__NavDelivery_GetResult_Event__Sequence * input,
  task_manager__action__NavDelivery_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_GetResult_Event * data =
      (task_manager__action__NavDelivery_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_GetResult_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "task_manager/action/detail/nav_delivery__functions.h"

bool
task_manager__action__NavDelivery_FeedbackMessage__init(task_manager__action__NavDelivery_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    task_manager__action__NavDelivery_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!task_manager__action__NavDelivery_Feedback__init(&msg->feedback)) {
    task_manager__action__NavDelivery_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
task_manager__action__NavDelivery_FeedbackMessage__fini(task_manager__action__NavDelivery_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  task_manager__action__NavDelivery_Feedback__fini(&msg->feedback);
}

bool
task_manager__action__NavDelivery_FeedbackMessage__are_equal(const task_manager__action__NavDelivery_FeedbackMessage * lhs, const task_manager__action__NavDelivery_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!task_manager__action__NavDelivery_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
task_manager__action__NavDelivery_FeedbackMessage__copy(
  const task_manager__action__NavDelivery_FeedbackMessage * input,
  task_manager__action__NavDelivery_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!task_manager__action__NavDelivery_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

task_manager__action__NavDelivery_FeedbackMessage *
task_manager__action__NavDelivery_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_FeedbackMessage * msg = (task_manager__action__NavDelivery_FeedbackMessage *)allocator.allocate(sizeof(task_manager__action__NavDelivery_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__action__NavDelivery_FeedbackMessage));
  bool success = task_manager__action__NavDelivery_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__action__NavDelivery_FeedbackMessage__destroy(task_manager__action__NavDelivery_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__action__NavDelivery_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__action__NavDelivery_FeedbackMessage__Sequence__init(task_manager__action__NavDelivery_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_FeedbackMessage * data = NULL;

  if (size) {
    data = (task_manager__action__NavDelivery_FeedbackMessage *)allocator.zero_allocate(size, sizeof(task_manager__action__NavDelivery_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__action__NavDelivery_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__action__NavDelivery_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
task_manager__action__NavDelivery_FeedbackMessage__Sequence__fini(task_manager__action__NavDelivery_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      task_manager__action__NavDelivery_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

task_manager__action__NavDelivery_FeedbackMessage__Sequence *
task_manager__action__NavDelivery_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__action__NavDelivery_FeedbackMessage__Sequence * array = (task_manager__action__NavDelivery_FeedbackMessage__Sequence *)allocator.allocate(sizeof(task_manager__action__NavDelivery_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__action__NavDelivery_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__action__NavDelivery_FeedbackMessage__Sequence__destroy(task_manager__action__NavDelivery_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__action__NavDelivery_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__action__NavDelivery_FeedbackMessage__Sequence__are_equal(const task_manager__action__NavDelivery_FeedbackMessage__Sequence * lhs, const task_manager__action__NavDelivery_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__action__NavDelivery_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__action__NavDelivery_FeedbackMessage__Sequence__copy(
  const task_manager__action__NavDelivery_FeedbackMessage__Sequence * input,
  task_manager__action__NavDelivery_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__action__NavDelivery_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__action__NavDelivery_FeedbackMessage * data =
      (task_manager__action__NavDelivery_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__action__NavDelivery_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__action__NavDelivery_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__action__NavDelivery_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
