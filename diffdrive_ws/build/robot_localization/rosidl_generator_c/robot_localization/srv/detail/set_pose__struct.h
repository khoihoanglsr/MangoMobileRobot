// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_localization:srv/SetPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_localization/srv/set_pose.h"


#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__STRUCT_H_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.h"

/// Struct defined in srv/SetPose in the package robot_localization.
typedef struct robot_localization__srv__SetPose_Request
{
  geometry_msgs__msg__PoseWithCovarianceStamped pose;
} robot_localization__srv__SetPose_Request;

// Struct for a sequence of robot_localization__srv__SetPose_Request.
typedef struct robot_localization__srv__SetPose_Request__Sequence
{
  robot_localization__srv__SetPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__SetPose_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SetPose in the package robot_localization.
typedef struct robot_localization__srv__SetPose_Response
{
  uint8_t structure_needs_at_least_one_member;
} robot_localization__srv__SetPose_Response;

// Struct for a sequence of robot_localization__srv__SetPose_Response.
typedef struct robot_localization__srv__SetPose_Response__Sequence
{
  robot_localization__srv__SetPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__SetPose_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robot_localization__srv__SetPose_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robot_localization__srv__SetPose_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetPose in the package robot_localization.
typedef struct robot_localization__srv__SetPose_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robot_localization__srv__SetPose_Request__Sequence request;
  robot_localization__srv__SetPose_Response__Sequence response;
} robot_localization__srv__SetPose_Event;

// Struct for a sequence of robot_localization__srv__SetPose_Event.
typedef struct robot_localization__srv__SetPose_Event__Sequence
{
  robot_localization__srv__SetPose_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__SetPose_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_POSE__STRUCT_H_
