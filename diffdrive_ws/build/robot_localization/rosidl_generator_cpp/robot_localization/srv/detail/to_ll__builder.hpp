// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/ToLL.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_localization/srv/to_ll.hpp"


#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_localization/srv/detail/to_ll__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToLL_Request_map_point
{
public:
  Init_ToLL_Request_map_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToLL_Request map_point(::robot_localization::srv::ToLL_Request::_map_point_type arg)
  {
    msg_.map_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToLL_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToLL_Request>()
{
  return robot_localization::srv::builder::Init_ToLL_Request_map_point();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToLL_Response_ll_point
{
public:
  Init_ToLL_Response_ll_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::ToLL_Response ll_point(::robot_localization::srv::ToLL_Response::_ll_point_type arg)
  {
    msg_.ll_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToLL_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToLL_Response>()
{
  return robot_localization::srv::builder::Init_ToLL_Response_ll_point();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_ToLL_Event_response
{
public:
  explicit Init_ToLL_Event_response(::robot_localization::srv::ToLL_Event & msg)
  : msg_(msg)
  {}
  ::robot_localization::srv::ToLL_Event response(::robot_localization::srv::ToLL_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::ToLL_Event msg_;
};

class Init_ToLL_Event_request
{
public:
  explicit Init_ToLL_Event_request(::robot_localization::srv::ToLL_Event & msg)
  : msg_(msg)
  {}
  Init_ToLL_Event_response request(::robot_localization::srv::ToLL_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ToLL_Event_response(msg_);
  }

private:
  ::robot_localization::srv::ToLL_Event msg_;
};

class Init_ToLL_Event_info
{
public:
  Init_ToLL_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToLL_Event_request info(::robot_localization::srv::ToLL_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ToLL_Event_request(msg_);
  }

private:
  ::robot_localization::srv::ToLL_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::ToLL_Event>()
{
  return robot_localization::srv::builder::Init_ToLL_Event_info();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__BUILDER_HPP_
