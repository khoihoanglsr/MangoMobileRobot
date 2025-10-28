// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_localization:srv/GetState.idl
// generated code does not contain a copyright notice
#include "robot_localization/srv/detail/get_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `time_stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `frame_id`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_localization__srv__GetState_Request__init(robot_localization__srv__GetState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // time_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->time_stamp)) {
    robot_localization__srv__GetState_Request__fini(msg);
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__init(&msg->frame_id)) {
    robot_localization__srv__GetState_Request__fini(msg);
    return false;
  }
  return true;
}

void
robot_localization__srv__GetState_Request__fini(robot_localization__srv__GetState_Request * msg)
{
  if (!msg) {
    return;
  }
  // time_stamp
  builtin_interfaces__msg__Time__fini(&msg->time_stamp);
  // frame_id
  rosidl_runtime_c__String__fini(&msg->frame_id);
}

bool
robot_localization__srv__GetState_Request__are_equal(const robot_localization__srv__GetState_Request * lhs, const robot_localization__srv__GetState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // time_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->time_stamp), &(rhs->time_stamp)))
  {
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->frame_id), &(rhs->frame_id)))
  {
    return false;
  }
  return true;
}

bool
robot_localization__srv__GetState_Request__copy(
  const robot_localization__srv__GetState_Request * input,
  robot_localization__srv__GetState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // time_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->time_stamp), &(output->time_stamp)))
  {
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__copy(
      &(input->frame_id), &(output->frame_id)))
  {
    return false;
  }
  return true;
}

robot_localization__srv__GetState_Request *
robot_localization__srv__GetState_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Request * msg = (robot_localization__srv__GetState_Request *)allocator.allocate(sizeof(robot_localization__srv__GetState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_localization__srv__GetState_Request));
  bool success = robot_localization__srv__GetState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_localization__srv__GetState_Request__destroy(robot_localization__srv__GetState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_localization__srv__GetState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_localization__srv__GetState_Request__Sequence__init(robot_localization__srv__GetState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Request * data = NULL;

  if (size) {
    data = (robot_localization__srv__GetState_Request *)allocator.zero_allocate(size, sizeof(robot_localization__srv__GetState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_localization__srv__GetState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_localization__srv__GetState_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_localization__srv__GetState_Request__Sequence__fini(robot_localization__srv__GetState_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_localization__srv__GetState_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_localization__srv__GetState_Request__Sequence *
robot_localization__srv__GetState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Request__Sequence * array = (robot_localization__srv__GetState_Request__Sequence *)allocator.allocate(sizeof(robot_localization__srv__GetState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_localization__srv__GetState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_localization__srv__GetState_Request__Sequence__destroy(robot_localization__srv__GetState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_localization__srv__GetState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_localization__srv__GetState_Request__Sequence__are_equal(const robot_localization__srv__GetState_Request__Sequence * lhs, const robot_localization__srv__GetState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_localization__srv__GetState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_localization__srv__GetState_Request__Sequence__copy(
  const robot_localization__srv__GetState_Request__Sequence * input,
  robot_localization__srv__GetState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_localization__srv__GetState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_localization__srv__GetState_Request * data =
      (robot_localization__srv__GetState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_localization__srv__GetState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_localization__srv__GetState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_localization__srv__GetState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
robot_localization__srv__GetState_Response__init(robot_localization__srv__GetState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // state
  // covariance
  return true;
}

void
robot_localization__srv__GetState_Response__fini(robot_localization__srv__GetState_Response * msg)
{
  if (!msg) {
    return;
  }
  // state
  // covariance
}

bool
robot_localization__srv__GetState_Response__are_equal(const robot_localization__srv__GetState_Response * lhs, const robot_localization__srv__GetState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  for (size_t i = 0; i < 15; ++i) {
    if (lhs->state[i] != rhs->state[i]) {
      return false;
    }
  }
  // covariance
  for (size_t i = 0; i < 225; ++i) {
    if (lhs->covariance[i] != rhs->covariance[i]) {
      return false;
    }
  }
  return true;
}

bool
robot_localization__srv__GetState_Response__copy(
  const robot_localization__srv__GetState_Response * input,
  robot_localization__srv__GetState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  for (size_t i = 0; i < 15; ++i) {
    output->state[i] = input->state[i];
  }
  // covariance
  for (size_t i = 0; i < 225; ++i) {
    output->covariance[i] = input->covariance[i];
  }
  return true;
}

robot_localization__srv__GetState_Response *
robot_localization__srv__GetState_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Response * msg = (robot_localization__srv__GetState_Response *)allocator.allocate(sizeof(robot_localization__srv__GetState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_localization__srv__GetState_Response));
  bool success = robot_localization__srv__GetState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_localization__srv__GetState_Response__destroy(robot_localization__srv__GetState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_localization__srv__GetState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_localization__srv__GetState_Response__Sequence__init(robot_localization__srv__GetState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Response * data = NULL;

  if (size) {
    data = (robot_localization__srv__GetState_Response *)allocator.zero_allocate(size, sizeof(robot_localization__srv__GetState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_localization__srv__GetState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_localization__srv__GetState_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_localization__srv__GetState_Response__Sequence__fini(robot_localization__srv__GetState_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_localization__srv__GetState_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_localization__srv__GetState_Response__Sequence *
robot_localization__srv__GetState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Response__Sequence * array = (robot_localization__srv__GetState_Response__Sequence *)allocator.allocate(sizeof(robot_localization__srv__GetState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_localization__srv__GetState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_localization__srv__GetState_Response__Sequence__destroy(robot_localization__srv__GetState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_localization__srv__GetState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_localization__srv__GetState_Response__Sequence__are_equal(const robot_localization__srv__GetState_Response__Sequence * lhs, const robot_localization__srv__GetState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_localization__srv__GetState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_localization__srv__GetState_Response__Sequence__copy(
  const robot_localization__srv__GetState_Response__Sequence * input,
  robot_localization__srv__GetState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_localization__srv__GetState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_localization__srv__GetState_Response * data =
      (robot_localization__srv__GetState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_localization__srv__GetState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_localization__srv__GetState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_localization__srv__GetState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "robot_localization/srv/detail/get_state__functions.h"

bool
robot_localization__srv__GetState_Event__init(robot_localization__srv__GetState_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    robot_localization__srv__GetState_Event__fini(msg);
    return false;
  }
  // request
  if (!robot_localization__srv__GetState_Request__Sequence__init(&msg->request, 0)) {
    robot_localization__srv__GetState_Event__fini(msg);
    return false;
  }
  // response
  if (!robot_localization__srv__GetState_Response__Sequence__init(&msg->response, 0)) {
    robot_localization__srv__GetState_Event__fini(msg);
    return false;
  }
  return true;
}

void
robot_localization__srv__GetState_Event__fini(robot_localization__srv__GetState_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  robot_localization__srv__GetState_Request__Sequence__fini(&msg->request);
  // response
  robot_localization__srv__GetState_Response__Sequence__fini(&msg->response);
}

bool
robot_localization__srv__GetState_Event__are_equal(const robot_localization__srv__GetState_Event * lhs, const robot_localization__srv__GetState_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!robot_localization__srv__GetState_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!robot_localization__srv__GetState_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
robot_localization__srv__GetState_Event__copy(
  const robot_localization__srv__GetState_Event * input,
  robot_localization__srv__GetState_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!robot_localization__srv__GetState_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!robot_localization__srv__GetState_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

robot_localization__srv__GetState_Event *
robot_localization__srv__GetState_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Event * msg = (robot_localization__srv__GetState_Event *)allocator.allocate(sizeof(robot_localization__srv__GetState_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_localization__srv__GetState_Event));
  bool success = robot_localization__srv__GetState_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_localization__srv__GetState_Event__destroy(robot_localization__srv__GetState_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_localization__srv__GetState_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_localization__srv__GetState_Event__Sequence__init(robot_localization__srv__GetState_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Event * data = NULL;

  if (size) {
    data = (robot_localization__srv__GetState_Event *)allocator.zero_allocate(size, sizeof(robot_localization__srv__GetState_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_localization__srv__GetState_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_localization__srv__GetState_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_localization__srv__GetState_Event__Sequence__fini(robot_localization__srv__GetState_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_localization__srv__GetState_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_localization__srv__GetState_Event__Sequence *
robot_localization__srv__GetState_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_localization__srv__GetState_Event__Sequence * array = (robot_localization__srv__GetState_Event__Sequence *)allocator.allocate(sizeof(robot_localization__srv__GetState_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_localization__srv__GetState_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_localization__srv__GetState_Event__Sequence__destroy(robot_localization__srv__GetState_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_localization__srv__GetState_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_localization__srv__GetState_Event__Sequence__are_equal(const robot_localization__srv__GetState_Event__Sequence * lhs, const robot_localization__srv__GetState_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_localization__srv__GetState_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_localization__srv__GetState_Event__Sequence__copy(
  const robot_localization__srv__GetState_Event__Sequence * input,
  robot_localization__srv__GetState_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_localization__srv__GetState_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_localization__srv__GetState_Event * data =
      (robot_localization__srv__GetState_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_localization__srv__GetState_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_localization__srv__GetState_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_localization__srv__GetState_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
