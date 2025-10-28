// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/ToggleFilterProcessing.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_localization/srv/toggle_filter_processing.hpp"


#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_localization/srv/detail/toggle_filter_processing__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToggleFilterProcessing_Request_on
{
public:
  Init_ToggleFilterProcessing_Request_on()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToggleFilterProcessing_Request on(::robot_localization::srv::ToggleFilterProcessing_Request::_on_type arg)
  {
    msg_.on = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToggleFilterProcessing_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToggleFilterProcessing_Request>()
{
  return robot_localization::srv::builder::Init_ToggleFilterProcessing_Request_on();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToggleFilterProcessing_Response_status
{
public:
  Init_ToggleFilterProcessing_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToggleFilterProcessing_Response status(::robot_localization::srv::ToggleFilterProcessing_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToggleFilterProcessing_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToggleFilterProcessing_Response>()
{
  return robot_localization::srv::builder::Init_ToggleFilterProcessing_Response_status();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToggleFilterProcessing_Event_response
{
public:
  explicit Init_ToggleFilterProcessing_Event_response(::robot_localization::srv::ToggleFilterProcessing_Event & msg)
  : msg_(msg)
  {}
  ::robot_localization::srv::ToggleFilterProcessing_Event response(::robot_localization::srv::ToggleFilterProcessing_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToggleFilterProcessing_Event msg_;
};

class Init_ToggleFilterProcessing_Event_request
{
public:
  explicit Init_ToggleFilterProcessing_Event_request(::robot_localization::srv::ToggleFilterProcessing_Event & msg)
  : msg_(msg)
  {}
  Init_ToggleFilterProcessing_Event_response request(::robot_localization::srv::ToggleFilterProcessing_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ToggleFilterProcessing_Event_response(msg_);
  }

private:
  ::robot_localization::srv::ToggleFilterProcessing_Event msg_;
};

class Init_ToggleFilterProcessing_Event_info
{
public:
  Init_ToggleFilterProcessing_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToggleFilterProcessing_Event_request info(::robot_localization::srv::ToggleFilterProcessing_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ToggleFilterProcessing_Event_request(msg_);
  }

private:
  ::robot_localization::srv::ToggleFilterProcessing_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToggleFilterProcessing_Event>()
{
  return robot_localization::srv::builder::Init_ToggleFilterProcessing_Event_info();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__BUILDER_HPP_
