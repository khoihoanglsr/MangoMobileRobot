// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/SetDatum.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_localization/srv/set_datum.hpp"


#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_localization/srv/detail/set_datum__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_SetDatum_Request_geo_pose
{
public:
  Init_SetDatum_Request_geo_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::SetDatum_Request geo_pose(::robot_localization::srv::SetDatum_Request::_geo_pose_type arg)
  {
    msg_.geo_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::SetDatum_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetDatum_Request>()
{
  return robot_localization::srv::builder::Init_SetDatum_Request_geo_pose();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetDatum_Response>()
{
  return ::robot_localization::srv::SetDatum_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_SetDatum_Event_response
{
public:
  explicit Init_SetDatum_Event_response(::robot_localization::srv::SetDatum_Event & msg)
  : msg_(msg)
  {}
  ::robot_localization::srv::SetDatum_Event response(::robot_localization::srv::SetDatum_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::SetDatum_Event msg_;
};

class Init_SetDatum_Event_request
{
public:
  explicit Init_SetDatum_Event_request(::robot_localization::srv::SetDatum_Event & msg)
  : msg_(msg)
  {}
  Init_SetDatum_Event_response request(::robot_localization::srv::SetDatum_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetDatum_Event_response(msg_);
  }

private:
  ::robot_localization::srv::SetDatum_Event msg_;
};

class Init_SetDatum_Event_info
{
public:
  Init_SetDatum_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetDatum_Event_request info(::robot_localization::srv::SetDatum_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetDatum_Event_request(msg_);
  }

private:
  ::robot_localization::srv::SetDatum_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetDatum_Event>()
{
  return robot_localization::srv::builder::Init_SetDatum_Event_info();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_DATUM__BUILDER_HPP_
