// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from task_manager:action/ManipulationTask.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_manager/action/manipulation_task.hpp"


#ifndef TASK_MANAGER__ACTION__DETAIL__MANIPULATION_TASK__BUILDER_HPP_
#define TASK_MANAGER__ACTION__DETAIL__MANIPULATION_TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "task_manager/action/detail/manipulation_task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_Goal_target_station
{
public:
  Init_ManipulationTask_Goal_target_station()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::task_manager::action::ManipulationTask_Goal target_station(::task_manager::action::ManipulationTask_Goal::_target_station_type arg)
  {
    msg_.target_station = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_Goal>()
{
  return task_manager::action::builder::Init_ManipulationTask_Goal_target_station();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_Result_error_msg
{
public:
  explicit Init_ManipulationTask_Result_error_msg(::task_manager::action::ManipulationTask_Result & msg)
  : msg_(msg)
  {}
  ::task_manager::action::ManipulationTask_Result error_msg(::task_manager::action::ManipulationTask_Result::_error_msg_type arg)
  {
    msg_.error_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_Result msg_;
};

class Init_ManipulationTask_Result_error_code
{
public:
  explicit Init_ManipulationTask_Result_error_code(::task_manager::action::ManipulationTask_Result & msg)
  : msg_(msg)
  {}
  Init_ManipulationTask_Result_error_msg error_code(::task_manager::action::ManipulationTask_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_ManipulationTask_Result_error_msg(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_Result msg_;
};

class Init_ManipulationTask_Result_success
{
public:
  Init_ManipulationTask_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulationTask_Result_error_code success(::task_manager::action::ManipulationTask_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ManipulationTask_Result_error_code(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_Result>()
{
  return task_manager::action::builder::Init_ManipulationTask_Result_success();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_Feedback_progress
{
public:
  Init_ManipulationTask_Feedback_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::task_manager::action::ManipulationTask_Feedback progress(::task_manager::action::ManipulationTask_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_Feedback>()
{
  return task_manager::action::builder::Init_ManipulationTask_Feedback_progress();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_SendGoal_Request_goal
{
public:
  explicit Init_ManipulationTask_SendGoal_Request_goal(::task_manager::action::ManipulationTask_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::task_manager::action::ManipulationTask_SendGoal_Request goal(::task_manager::action::ManipulationTask_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_SendGoal_Request msg_;
};

class Init_ManipulationTask_SendGoal_Request_goal_id
{
public:
  Init_ManipulationTask_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulationTask_SendGoal_Request_goal goal_id(::task_manager::action::ManipulationTask_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ManipulationTask_SendGoal_Request_goal(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_SendGoal_Request>()
{
  return task_manager::action::builder::Init_ManipulationTask_SendGoal_Request_goal_id();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_SendGoal_Response_stamp
{
public:
  explicit Init_ManipulationTask_SendGoal_Response_stamp(::task_manager::action::ManipulationTask_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::task_manager::action::ManipulationTask_SendGoal_Response stamp(::task_manager::action::ManipulationTask_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_SendGoal_Response msg_;
};

class Init_ManipulationTask_SendGoal_Response_accepted
{
public:
  Init_ManipulationTask_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulationTask_SendGoal_Response_stamp accepted(::task_manager::action::ManipulationTask_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ManipulationTask_SendGoal_Response_stamp(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_SendGoal_Response>()
{
  return task_manager::action::builder::Init_ManipulationTask_SendGoal_Response_accepted();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_SendGoal_Event_response
{
public:
  explicit Init_ManipulationTask_SendGoal_Event_response(::task_manager::action::ManipulationTask_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::task_manager::action::ManipulationTask_SendGoal_Event response(::task_manager::action::ManipulationTask_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_SendGoal_Event msg_;
};

class Init_ManipulationTask_SendGoal_Event_request
{
public:
  explicit Init_ManipulationTask_SendGoal_Event_request(::task_manager::action::ManipulationTask_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_ManipulationTask_SendGoal_Event_response request(::task_manager::action::ManipulationTask_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ManipulationTask_SendGoal_Event_response(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_SendGoal_Event msg_;
};

class Init_ManipulationTask_SendGoal_Event_info
{
public:
  Init_ManipulationTask_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulationTask_SendGoal_Event_request info(::task_manager::action::ManipulationTask_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ManipulationTask_SendGoal_Event_request(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_SendGoal_Event>()
{
  return task_manager::action::builder::Init_ManipulationTask_SendGoal_Event_info();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_GetResult_Request_goal_id
{
public:
  Init_ManipulationTask_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::task_manager::action::ManipulationTask_GetResult_Request goal_id(::task_manager::action::ManipulationTask_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_GetResult_Request>()
{
  return task_manager::action::builder::Init_ManipulationTask_GetResult_Request_goal_id();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_GetResult_Response_result
{
public:
  explicit Init_ManipulationTask_GetResult_Response_result(::task_manager::action::ManipulationTask_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::task_manager::action::ManipulationTask_GetResult_Response result(::task_manager::action::ManipulationTask_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_GetResult_Response msg_;
};

class Init_ManipulationTask_GetResult_Response_status
{
public:
  Init_ManipulationTask_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulationTask_GetResult_Response_result status(::task_manager::action::ManipulationTask_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ManipulationTask_GetResult_Response_result(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_GetResult_Response>()
{
  return task_manager::action::builder::Init_ManipulationTask_GetResult_Response_status();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_GetResult_Event_response
{
public:
  explicit Init_ManipulationTask_GetResult_Event_response(::task_manager::action::ManipulationTask_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::task_manager::action::ManipulationTask_GetResult_Event response(::task_manager::action::ManipulationTask_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_GetResult_Event msg_;
};

class Init_ManipulationTask_GetResult_Event_request
{
public:
  explicit Init_ManipulationTask_GetResult_Event_request(::task_manager::action::ManipulationTask_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_ManipulationTask_GetResult_Event_response request(::task_manager::action::ManipulationTask_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ManipulationTask_GetResult_Event_response(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_GetResult_Event msg_;
};

class Init_ManipulationTask_GetResult_Event_info
{
public:
  Init_ManipulationTask_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulationTask_GetResult_Event_request info(::task_manager::action::ManipulationTask_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ManipulationTask_GetResult_Event_request(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_GetResult_Event>()
{
  return task_manager::action::builder::Init_ManipulationTask_GetResult_Event_info();
}

}  // namespace task_manager


namespace task_manager
{

namespace action
{

namespace builder
{

class Init_ManipulationTask_FeedbackMessage_feedback
{
public:
  explicit Init_ManipulationTask_FeedbackMessage_feedback(::task_manager::action::ManipulationTask_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::task_manager::action::ManipulationTask_FeedbackMessage feedback(::task_manager::action::ManipulationTask_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_FeedbackMessage msg_;
};

class Init_ManipulationTask_FeedbackMessage_goal_id
{
public:
  Init_ManipulationTask_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulationTask_FeedbackMessage_feedback goal_id(::task_manager::action::ManipulationTask_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ManipulationTask_FeedbackMessage_feedback(msg_);
  }

private:
  ::task_manager::action::ManipulationTask_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_manager::action::ManipulationTask_FeedbackMessage>()
{
  return task_manager::action::builder::Init_ManipulationTask_FeedbackMessage_goal_id();
}

}  // namespace task_manager

#endif  // TASK_MANAGER__ACTION__DETAIL__MANIPULATION_TASK__BUILDER_HPP_
