// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_localization:srv/ToggleFilterProcessing.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_localization/srv/toggle_filter_processing.h"


#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_H_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ToggleFilterProcessing in the package robot_localization.
typedef struct robot_localization__srv__ToggleFilterProcessing_Request
{
  bool on;
} robot_localization__srv__ToggleFilterProcessing_Request;

// Struct for a sequence of robot_localization__srv__ToggleFilterProcessing_Request.
typedef struct robot_localization__srv__ToggleFilterProcessing_Request__Sequence
{
  robot_localization__srv__ToggleFilterProcessing_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToggleFilterProcessing_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/ToggleFilterProcessing in the package robot_localization.
typedef struct robot_localization__srv__ToggleFilterProcessing_Response
{
  bool status;
} robot_localization__srv__ToggleFilterProcessing_Response;

// Struct for a sequence of robot_localization__srv__ToggleFilterProcessing_Response.
typedef struct robot_localization__srv__ToggleFilterProcessing_Response__Sequence
{
  robot_localization__srv__ToggleFilterProcessing_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToggleFilterProcessing_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robot_localization__srv__ToggleFilterProcessing_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robot_localization__srv__ToggleFilterProcessing_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ToggleFilterProcessing in the package robot_localization.
typedef struct robot_localization__srv__ToggleFilterProcessing_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robot_localization__srv__ToggleFilterProcessing_Request__Sequence request;
  robot_localization__srv__ToggleFilterProcessing_Response__Sequence response;
} robot_localization__srv__ToggleFilterProcessing_Event;

// Struct for a sequence of robot_localization__srv__ToggleFilterProcessing_Event.
typedef struct robot_localization__srv__ToggleFilterProcessing_Event__Sequence
{
  robot_localization__srv__ToggleFilterProcessing_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToggleFilterProcessing_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TOGGLE_FILTER_PROCESSING__STRUCT_H_
