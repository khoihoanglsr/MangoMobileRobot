// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_localization:srv/SetUTMZone.idl
// generated code does not contain a copyright notice

#include "robot_localization/srv/detail/set_utm_zone__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetUTMZone__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x54, 0x73, 0xd7, 0x59, 0x05, 0x6a, 0xbe, 0xa2,
      0x8c, 0xb6, 0x5a, 0x7c, 0xd8, 0x09, 0x51, 0xec,
      0x06, 0x59, 0x23, 0x34, 0x90, 0xc0, 0xc9, 0x9f,
      0x8f, 0x4d, 0xab, 0x63, 0x36, 0x17, 0x75, 0x65,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetUTMZone_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x48, 0x1a, 0xfa, 0x24, 0x56, 0x8f, 0xc1, 0x40,
      0xa3, 0x59, 0x91, 0xb1, 0x55, 0x53, 0xf3, 0x19,
      0x4e, 0x5f, 0x80, 0xa5, 0x23, 0xe4, 0xaf, 0x7c,
      0x45, 0xb5, 0xc6, 0x13, 0xbe, 0xee, 0xea, 0xd5,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetUTMZone_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1d, 0x9e, 0xd5, 0x69, 0xda, 0xda, 0x2a, 0x7d,
      0x56, 0xa6, 0x4d, 0x8e, 0x70, 0x45, 0x41, 0xcf,
      0xb1, 0x2a, 0xa7, 0xe9, 0x5c, 0xc8, 0x2a, 0x6b,
      0x18, 0x22, 0xb4, 0x8f, 0x7b, 0xcb, 0x44, 0x3a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetUTMZone_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe1, 0xdf, 0x83, 0xd3, 0xcd, 0xeb, 0xd2, 0xaa,
      0x3b, 0x2b, 0xa4, 0xd3, 0x61, 0xa5, 0x02, 0x1b,
      0x0f, 0xbb, 0x5f, 0x53, 0x29, 0x8a, 0x2b, 0x59,
      0x6c, 0xe0, 0xf0, 0x2b, 0x24, 0x1d, 0xb1, 0x6a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char robot_localization__srv__SetUTMZone__TYPE_NAME[] = "robot_localization/srv/SetUTMZone";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char robot_localization__srv__SetUTMZone_Event__TYPE_NAME[] = "robot_localization/srv/SetUTMZone_Event";
static char robot_localization__srv__SetUTMZone_Request__TYPE_NAME[] = "robot_localization/srv/SetUTMZone_Request";
static char robot_localization__srv__SetUTMZone_Response__TYPE_NAME[] = "robot_localization/srv/SetUTMZone_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robot_localization__srv__SetUTMZone__FIELD_NAME__request_message[] = "request_message";
static char robot_localization__srv__SetUTMZone__FIELD_NAME__response_message[] = "response_message";
static char robot_localization__srv__SetUTMZone__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetUTMZone__FIELDS[] = {
  {
    {robot_localization__srv__SetUTMZone__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_localization__srv__SetUTMZone_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_localization__srv__SetUTMZone_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_localization__srv__SetUTMZone_Event__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_localization__srv__SetUTMZone__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone_Event__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_localization__srv__SetUTMZone__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetUTMZone__TYPE_NAME, 33, 33},
      {robot_localization__srv__SetUTMZone__FIELDS, 3, 3},
    },
    {robot_localization__srv__SetUTMZone__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_localization__srv__SetUTMZone_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_localization__srv__SetUTMZone_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = robot_localization__srv__SetUTMZone_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_localization__srv__SetUTMZone_Request__FIELD_NAME__utm_zone[] = "utm_zone";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetUTMZone_Request__FIELDS[] = {
  {
    {robot_localization__srv__SetUTMZone_Request__FIELD_NAME__utm_zone, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_localization__srv__SetUTMZone_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetUTMZone_Request__TYPE_NAME, 41, 41},
      {robot_localization__srv__SetUTMZone_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_localization__srv__SetUTMZone_Response__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetUTMZone_Response__FIELDS[] = {
  {
    {robot_localization__srv__SetUTMZone_Response__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_localization__srv__SetUTMZone_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetUTMZone_Response__TYPE_NAME, 42, 42},
      {robot_localization__srv__SetUTMZone_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_localization__srv__SetUTMZone_Event__FIELD_NAME__info[] = "info";
static char robot_localization__srv__SetUTMZone_Event__FIELD_NAME__request[] = "request";
static char robot_localization__srv__SetUTMZone_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetUTMZone_Event__FIELDS[] = {
  {
    {robot_localization__srv__SetUTMZone_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_localization__srv__SetUTMZone_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_localization__srv__SetUTMZone_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_localization__srv__SetUTMZone_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetUTMZone_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_localization__srv__SetUTMZone_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetUTMZone_Event__TYPE_NAME, 39, 39},
      {robot_localization__srv__SetUTMZone_Event__FIELDS, 3, 3},
    },
    {robot_localization__srv__SetUTMZone_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robot_localization__srv__SetUTMZone_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robot_localization__srv__SetUTMZone_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Set an UTM zone navsat_transform should stick to.\n"
  "# Example: 31U\n"
  "string utm_zone\n"
  "---";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetUTMZone__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetUTMZone__TYPE_NAME, 33, 33},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 87, 87},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetUTMZone_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetUTMZone_Request__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetUTMZone_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetUTMZone_Response__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetUTMZone_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetUTMZone_Event__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetUTMZone__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetUTMZone__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_localization__srv__SetUTMZone_Event__get_individual_type_description_source(NULL);
    sources[3] = *robot_localization__srv__SetUTMZone_Request__get_individual_type_description_source(NULL);
    sources[4] = *robot_localization__srv__SetUTMZone_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetUTMZone_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetUTMZone_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetUTMZone_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetUTMZone_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetUTMZone_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetUTMZone_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robot_localization__srv__SetUTMZone_Request__get_individual_type_description_source(NULL);
    sources[3] = *robot_localization__srv__SetUTMZone_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
