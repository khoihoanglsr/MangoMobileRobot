// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/SetPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_localization/srv/set_pose.hpp"


#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_localization/srv/detail/set_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_SetPose_Request_pose
{
public:
  Init_SetPose_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::SetPose_Request pose(::robot_localization::srv::SetPose_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::SetPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetPose_Request>()
{
  return robot_localization::srv::builder::Init_SetPose_Request_pose();
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
auto build<::robot_localization::srv::SetPose_Response>()
{
  return ::robot_localization::srv::SetPose_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_SetPose_Event_response
{
public:
  explicit Init_SetPose_Event_response(::robot_localization::srv::SetPose_Event & msg)
  : msg_(msg)
  {}
  ::robot_localization::srv::SetPose_Event response(::robot_localization::srv::SetPose_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::SetPose_Event msg_;
};

class Init_SetPose_Event_request
{
public:
  explicit Init_SetPose_Event_request(::robot_localization::srv::SetPose_Event & msg)
  : msg_(msg)
  {}
  Init_SetPose_Event_response request(::robot_localization::srv::SetPose_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetPose_Event_response(msg_);
  }

private:
  ::robot_localization::srv::SetPose_Event msg_;
};

class Init_SetPose_Event_info
{
public:
  Init_SetPose_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPose_Event_request info(::robot_localization::srv::SetPose_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetPose_Event_request(msg_);
  }

private:
  ::robot_localization::srv::SetPose_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetPose_Event>()
{
  return robot_localization::srv::builder::Init_SetPose_Event_info();
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__BUILDER_HPP_
