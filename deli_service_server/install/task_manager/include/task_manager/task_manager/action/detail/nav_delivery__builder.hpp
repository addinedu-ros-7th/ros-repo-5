// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from task_manager:action/NavDelivery.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/action/nav_delivery.hpp"


#ifndef TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__BUILDER_HPP_
#define TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "task_manager/action/detail/nav_delivery__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_Goal_stations
{
public:
  Init_NavDelivery_Goal_stations()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::task_manager::action::NavDelivery_Goal stations(::task_manager::action::NavDelivery_Goal::_stations_type arg)
  {
    msg_.stations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_Goal>()
{
  return task_manager::action::builder::Init_NavDelivery_Goal_stations();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_Result_error_msg
{
public:
  explicit Init_NavDelivery_Result_error_msg(::task_manager::action::NavDelivery_Result & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_Result error_msg(::task_manager::action::NavDelivery_Result::_error_msg_type arg)
  {
    msg_.error_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_Result msg_;
};

class Init_NavDelivery_Result_error_code
{
public:
  explicit Init_NavDelivery_Result_error_code(::task_manager::action::NavDelivery_Result & msg)
  : msg_(msg)
  {}
  Init_NavDelivery_Result_error_msg error_code(::task_manager::action::NavDelivery_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_NavDelivery_Result_error_msg(msg_);
  }

private:
  ::task_manager::action::NavDelivery_Result msg_;
};

class Init_NavDelivery_Result_success
{
public:
  Init_NavDelivery_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_Result_error_code success(::task_manager::action::NavDelivery_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_NavDelivery_Result_error_code(msg_);
  }

private:
  ::task_manager::action::NavDelivery_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_Result>()
{
  return task_manager::action::builder::Init_NavDelivery_Result_success();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_Feedback_distance_remaining
{
public:
  explicit Init_NavDelivery_Feedback_distance_remaining(::task_manager::action::NavDelivery_Feedback & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_Feedback distance_remaining(::task_manager::action::NavDelivery_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_Feedback msg_;
};

class Init_NavDelivery_Feedback_current_pose
{
public:
  Init_NavDelivery_Feedback_current_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_Feedback_distance_remaining current_pose(::task_manager::action::NavDelivery_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_NavDelivery_Feedback_distance_remaining(msg_);
  }

private:
  ::task_manager::action::NavDelivery_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_Feedback>()
{
  return task_manager::action::builder::Init_NavDelivery_Feedback_current_pose();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_SendGoal_Request_goal
{
public:
  explicit Init_NavDelivery_SendGoal_Request_goal(::task_manager::action::NavDelivery_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_SendGoal_Request goal(::task_manager::action::NavDelivery_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_SendGoal_Request msg_;
};

class Init_NavDelivery_SendGoal_Request_goal_id
{
public:
  Init_NavDelivery_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_SendGoal_Request_goal goal_id(::task_manager::action::NavDelivery_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavDelivery_SendGoal_Request_goal(msg_);
  }

private:
  ::task_manager::action::NavDelivery_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_SendGoal_Request>()
{
  return task_manager::action::builder::Init_NavDelivery_SendGoal_Request_goal_id();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_SendGoal_Response_stamp
{
public:
  explicit Init_NavDelivery_SendGoal_Response_stamp(::task_manager::action::NavDelivery_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_SendGoal_Response stamp(::task_manager::action::NavDelivery_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_SendGoal_Response msg_;
};

class Init_NavDelivery_SendGoal_Response_accepted
{
public:
  Init_NavDelivery_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_SendGoal_Response_stamp accepted(::task_manager::action::NavDelivery_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_NavDelivery_SendGoal_Response_stamp(msg_);
  }

private:
  ::task_manager::action::NavDelivery_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_SendGoal_Response>()
{
  return task_manager::action::builder::Init_NavDelivery_SendGoal_Response_accepted();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_SendGoal_Event_response
{
public:
  explicit Init_NavDelivery_SendGoal_Event_response(::task_manager::action::NavDelivery_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_SendGoal_Event response(::task_manager::action::NavDelivery_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_SendGoal_Event msg_;
};

class Init_NavDelivery_SendGoal_Event_request
{
public:
  explicit Init_NavDelivery_SendGoal_Event_request(::task_manager::action::NavDelivery_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_NavDelivery_SendGoal_Event_response request(::task_manager::action::NavDelivery_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_NavDelivery_SendGoal_Event_response(msg_);
  }

private:
  ::task_manager::action::NavDelivery_SendGoal_Event msg_;
};

class Init_NavDelivery_SendGoal_Event_info
{
public:
  Init_NavDelivery_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_SendGoal_Event_request info(::task_manager::action::NavDelivery_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_NavDelivery_SendGoal_Event_request(msg_);
  }

private:
  ::task_manager::action::NavDelivery_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_SendGoal_Event>()
{
  return task_manager::action::builder::Init_NavDelivery_SendGoal_Event_info();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_GetResult_Request_goal_id
{
public:
  Init_NavDelivery_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::task_manager::action::NavDelivery_GetResult_Request goal_id(::task_manager::action::NavDelivery_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_GetResult_Request>()
{
  return task_manager::action::builder::Init_NavDelivery_GetResult_Request_goal_id();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_GetResult_Response_result
{
public:
  explicit Init_NavDelivery_GetResult_Response_result(::task_manager::action::NavDelivery_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_GetResult_Response result(::task_manager::action::NavDelivery_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_GetResult_Response msg_;
};

class Init_NavDelivery_GetResult_Response_status
{
public:
  Init_NavDelivery_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_GetResult_Response_result status(::task_manager::action::NavDelivery_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavDelivery_GetResult_Response_result(msg_);
  }

private:
  ::task_manager::action::NavDelivery_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_GetResult_Response>()
{
  return task_manager::action::builder::Init_NavDelivery_GetResult_Response_status();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_GetResult_Event_response
{
public:
  explicit Init_NavDelivery_GetResult_Event_response(::task_manager::action::NavDelivery_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_GetResult_Event response(::task_manager::action::NavDelivery_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_GetResult_Event msg_;
};

class Init_NavDelivery_GetResult_Event_request
{
public:
  explicit Init_NavDelivery_GetResult_Event_request(::task_manager::action::NavDelivery_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_NavDelivery_GetResult_Event_response request(::task_manager::action::NavDelivery_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_NavDelivery_GetResult_Event_response(msg_);
  }

private:
  ::task_manager::action::NavDelivery_GetResult_Event msg_;
};

class Init_NavDelivery_GetResult_Event_info
{
public:
  Init_NavDelivery_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_GetResult_Event_request info(::task_manager::action::NavDelivery_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_NavDelivery_GetResult_Event_request(msg_);
  }

private:
  ::task_manager::action::NavDelivery_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_GetResult_Event>()
{
  return task_manager::action::builder::Init_NavDelivery_GetResult_Event_info();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_NavDelivery_FeedbackMessage_feedback
{
public:
  explicit Init_NavDelivery_FeedbackMessage_feedback(::task_manager::action::NavDelivery_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::task_manager::action::NavDelivery_FeedbackMessage feedback(::task_manager::action::NavDelivery_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::NavDelivery_FeedbackMessage msg_;
};

class Init_NavDelivery_FeedbackMessage_goal_id
{
public:
  Init_NavDelivery_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDelivery_FeedbackMessage_feedback goal_id(::task_manager::action::NavDelivery_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavDelivery_FeedbackMessage_feedback(msg_);
  }

private:
  ::task_manager::action::NavDelivery_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::NavDelivery_FeedbackMessage>()
{
  return task_manager::action::builder::Init_NavDelivery_FeedbackMessage_goal_id();
}

}  // namespace task_manager

#endif  // TASK_MANAGER__ACTION__DETAIL__NAV_DELIVERY__BUILDER_HPP_
