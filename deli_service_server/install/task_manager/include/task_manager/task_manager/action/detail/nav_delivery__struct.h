// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from task_manager:action/NavDelivery.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/action/nav_delivery.h"


#ifndef TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__STRUCT_H_
#define TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stations'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_Goal
{
  /// goal
  rosidl_runtime_c__String__Sequence stations;
} task_manager__action__NavDelivery_Goal;

// Struct for a sequence of task_manager__action__NavDelivery_Goal.
typedef struct task_manager__action__NavDelivery_Goal__Sequence
{
  task_manager__action__NavDelivery_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'error_msg'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_Result
{
  bool success;
  int32_t error_code;
  rosidl_runtime_c__String error_msg;
} task_manager__action__NavDelivery_Result;

// Struct for a sequence of task_manager__action__NavDelivery_Result.
typedef struct task_manager__action__NavDelivery_Result__Sequence
{
  task_manager__action__NavDelivery_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_Feedback
{
  geometry_msgs__msg__PoseStamped current_pose;
  float distance_remaining;
} task_manager__action__NavDelivery_Feedback;

// Struct for a sequence of task_manager__action__NavDelivery_Feedback.
typedef struct task_manager__action__NavDelivery_Feedback__Sequence
{
  task_manager__action__NavDelivery_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "task_manager/action/detail/nav_delivery__struct.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  task_manager__action__NavDelivery_Goal goal;
} task_manager__action__NavDelivery_SendGoal_Request;

// Struct for a sequence of task_manager__action__NavDelivery_SendGoal_Request.
typedef struct task_manager__action__NavDelivery_SendGoal_Request__Sequence
{
  task_manager__action__NavDelivery_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} task_manager__action__NavDelivery_SendGoal_Response;

// Struct for a sequence of task_manager__action__NavDelivery_SendGoal_Response.
typedef struct task_manager__action__NavDelivery_SendGoal_Response__Sequence
{
  task_manager__action__NavDelivery_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  task_manager__action__NavDelivery_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  task_manager__action__NavDelivery_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  task_manager__action__NavDelivery_SendGoal_Request__Sequence request;
  task_manager__action__NavDelivery_SendGoal_Response__Sequence response;
} task_manager__action__NavDelivery_SendGoal_Event;

// Struct for a sequence of task_manager__action__NavDelivery_SendGoal_Event.
typedef struct task_manager__action__NavDelivery_SendGoal_Event__Sequence
{
  task_manager__action__NavDelivery_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} task_manager__action__NavDelivery_GetResult_Request;

// Struct for a sequence of task_manager__action__NavDelivery_GetResult_Request.
typedef struct task_manager__action__NavDelivery_GetResult_Request__Sequence
{
  task_manager__action__NavDelivery_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "task_manager/action/detail/nav_delivery__struct.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_GetResult_Response
{
  int8_t status;
  task_manager__action__NavDelivery_Result result;
} task_manager__action__NavDelivery_GetResult_Response;

// Struct for a sequence of task_manager__action__NavDelivery_GetResult_Response.
typedef struct task_manager__action__NavDelivery_GetResult_Response__Sequence
{
  task_manager__action__NavDelivery_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  task_manager__action__NavDelivery_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  task_manager__action__NavDelivery_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  task_manager__action__NavDelivery_GetResult_Request__Sequence request;
  task_manager__action__NavDelivery_GetResult_Response__Sequence response;
} task_manager__action__NavDelivery_GetResult_Event;

// Struct for a sequence of task_manager__action__NavDelivery_GetResult_Event.
typedef struct task_manager__action__NavDelivery_GetResult_Event__Sequence
{
  task_manager__action__NavDelivery_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "task_manager/action/detail/nav_delivery__struct.h"

/// Struct defined in action/NavDelivery in the package task_manager.
typedef struct task_manager__action__NavDelivery_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  task_manager__action__NavDelivery_Feedback feedback;
} task_manager__action__NavDelivery_FeedbackMessage;

// Struct for a sequence of task_manager__action__NavDelivery_FeedbackMessage.
typedef struct task_manager__action__NavDelivery_FeedbackMessage__Sequence
{
  task_manager__action__NavDelivery_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__action__NavDelivery_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__STRUCT_H_
