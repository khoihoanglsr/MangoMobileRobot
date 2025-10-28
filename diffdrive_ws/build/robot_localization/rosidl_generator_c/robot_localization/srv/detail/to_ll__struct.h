// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_localization:srv/ToLL.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_localization/srv/to_ll.h"


#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__STRUCT_H_
#define ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'map_point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/ToLL in the package robot_localization.
typedef struct robot_localization__srv__ToLL_Request
{
  geometry_msgs__msg__Point map_point;
} robot_localization__srv__ToLL_Request;

// Struct for a sequence of robot_localization__srv__ToLL_Request.
typedef struct robot_localization__srv__ToLL_Request__Sequence
{
  robot_localization__srv__ToLL_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToLL_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'll_point'
#include "geographic_msgs/msg/detail/geo_point__struct.h"

/// Struct defined in srv/ToLL in the package robot_localization.
typedef struct robot_localization__srv__ToLL_Response
{
  geographic_msgs__msg__GeoPoint ll_point;
} robot_localization__srv__ToLL_Response;

// Struct for a sequence of robot_localization__srv__ToLL_Response.
typedef struct robot_localization__srv__ToLL_Response__Sequence
{
  robot_localization__srv__ToLL_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToLL_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robot_localization__srv__ToLL_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robot_localization__srv__ToLL_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ToLL in the package robot_localization.
typedef struct robot_localization__srv__ToLL_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robot_localization__srv__ToLL_Request__Sequence request;
  robot_localization__srv__ToLL_Response__Sequence response;
} robot_localization__srv__ToLL_Event;

// Struct for a sequence of robot_localization__srv__ToLL_Event.
typedef struct robot_localization__srv__ToLL_Event__Sequence
{
  robot_localization__srv__ToLL_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_localization__srv__ToLL_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__TO_LL__STRUCT_H_
