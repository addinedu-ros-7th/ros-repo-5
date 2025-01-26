// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from task_manager:msg/Task.idl
// generated code does not contain a copyright notice
#include "task_manager/msg/detail/task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `item`
#include "rosidl_runtime_c/string_functions.h"

bool
task_manager__msg__Task__init(task_manager__msg__Task * msg)
{
  if (!msg) {
    return false;
  }
  // item
  if (!rosidl_runtime_c__String__init(&msg->item)) {
    task_manager__msg__Task__fini(msg);
    return false;
  }
  // quantity
  return true;
}

void
task_manager__msg__Task__fini(task_manager__msg__Task * msg)
{
  if (!msg) {
    return;
  }
  // item
  rosidl_runtime_c__String__fini(&msg->item);
  // quantity
}

bool
task_manager__msg__Task__are_equal(const task_manager__msg__Task * lhs, const task_manager__msg__Task * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // item
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->item), &(rhs->item)))
  {
    return false;
  }
  // quantity
  if (lhs->quantity != rhs->quantity) {
    return false;
  }
  return true;
}

bool
task_manager__msg__Task__copy(
  const task_manager__msg__Task * input,
  task_manager__msg__Task * output)
{
  if (!input || !output) {
    return false;
  }
  // item
  if (!rosidl_runtime_c__String__copy(
      &(input->item), &(output->item)))
  {
    return false;
  }
  // quantity
  output->quantity = input->quantity;
  return true;
}

task_manager__msg__Task *
task_manager__msg__Task__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__msg__Task * msg = (task_manager__msg__Task *)allocator.allocate(sizeof(task_manager__msg__Task), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(task_manager__msg__Task));
  bool success = task_manager__msg__Task__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
task_manager__msg__Task__destroy(task_manager__msg__Task * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    task_manager__msg__Task__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
task_manager__msg__Task__Sequence__init(task_manager__msg__Task__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__msg__Task * data = NULL;

  if (size) {
    data = (task_manager__msg__Task *)allocator.zero_allocate(size, sizeof(task_manager__msg__Task), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = task_manager__msg__Task__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        task_manager__msg__Task__fini(&data[i - 1]);
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
task_manager__msg__Task__Sequence__fini(task_manager__msg__Task__Sequence * array)
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
      task_manager__msg__Task__fini(&array->data[i]);
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

task_manager__msg__Task__Sequence *
task_manager__msg__Task__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  task_manager__msg__Task__Sequence * array = (task_manager__msg__Task__Sequence *)allocator.allocate(sizeof(task_manager__msg__Task__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = task_manager__msg__Task__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
task_manager__msg__Task__Sequence__destroy(task_manager__msg__Task__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    task_manager__msg__Task__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
task_manager__msg__Task__Sequence__are_equal(const task_manager__msg__Task__Sequence * lhs, const task_manager__msg__Task__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!task_manager__msg__Task__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
task_manager__msg__Task__Sequence__copy(
  const task_manager__msg__Task__Sequence * input,
  task_manager__msg__Task__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(task_manager__msg__Task);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    task_manager__msg__Task * data =
      (task_manager__msg__Task *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!task_manager__msg__Task__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          task_manager__msg__Task__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!task_manager__msg__Task__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
