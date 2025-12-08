// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_localization:srv/SetDatum.idl
// generated code does not contain a copyright notice

#include "robot_localization/srv/detail/set_datum__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetDatum__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdb, 0xb2, 0x6d, 0x6f, 0xa5, 0x55, 0x49, 0x1d,
      0x14, 0xd1, 0xde, 0x3f, 0xd0, 0x30, 0x8f, 0x10,
      0xbc, 0x13, 0x98, 0x01, 0xb8, 0x0c, 0xcb, 0x71,
      0x51, 0x14, 0x25, 0xff, 0xc3, 0x36, 0xc5, 0x73,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetDatum_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc9, 0xe7, 0x60, 0xea, 0x18, 0x1e, 0xfb, 0x63,
      0x4d, 0xe6, 0xf7, 0xf2, 0xb2, 0xbe, 0xd2, 0x62,
      0xdc, 0xe1, 0x5c, 0xb7, 0x2b, 0x17, 0xf9, 0xac,
      0x63, 0xbc, 0x46, 0x0a, 0x30, 0xbf, 0xd7, 0x88,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetDatum_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x63, 0x77, 0x18, 0x39, 0x6a, 0xb0, 0x14, 0x52,
      0xe7, 0x2b, 0x15, 0x5e, 0x9c, 0x6d, 0x6c, 0xb1,
      0x82, 0x30, 0x61, 0x9a, 0x2b, 0x9d, 0xeb, 0x46,
      0x35, 0x8f, 0xd7, 0x8d, 0x0f, 0x7f, 0xa2, 0x98,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robot_localization
const rosidl_type_hash_t *
robot_localization__srv__SetDatum_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6d, 0x98, 0x17, 0x20, 0xd1, 0x33, 0x65, 0x80,
      0x40, 0xbd, 0x76, 0xe4, 0xd1, 0x2a, 0x71, 0x40,
      0x64, 0x49, 0x01, 0xcb, 0x1e, 0xb4, 0x99, 0x43,
      0x56, 0xa9, 0xc1, 0xea, 0xca, 0xd0, 0xf7, 0xb7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geographic_msgs/msg/detail/geo_point__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "geographic_msgs/msg/detail/geo_pose__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geographic_msgs__msg__GeoPoint__EXPECTED_HASH = {1, {
    0x60, 0x2c, 0x64, 0x7c, 0xf8, 0x7b, 0xbe, 0x95,
    0xa0, 0x23, 0x1b, 0x45, 0x9b, 0x5a, 0xc3, 0x00,
    0x51, 0x87, 0x5d, 0x07, 0xc0, 0xc7, 0x40, 0xb3,
    0x1a, 0xd7, 0xa8, 0xc4, 0x66, 0x2b, 0x97, 0x1e,
  }};
static const rosidl_type_hash_t geographic_msgs__msg__GeoPose__EXPECTED_HASH = {1, {
    0x80, 0x52, 0x78, 0x6d, 0x76, 0x63, 0xe2, 0x25,
    0xe3, 0x89, 0x26, 0xf7, 0xd0, 0x57, 0x16, 0x04,
    0x6c, 0x93, 0x15, 0x72, 0xc2, 0x5e, 0x55, 0x7d,
    0x4f, 0x70, 0xdc, 0xa5, 0x5b, 0x97, 0x26, 0x26,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char robot_localization__srv__SetDatum__TYPE_NAME[] = "robot_localization/srv/SetDatum";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geographic_msgs__msg__GeoPoint__TYPE_NAME[] = "geographic_msgs/msg/GeoPoint";
static char geographic_msgs__msg__GeoPose__TYPE_NAME[] = "geographic_msgs/msg/GeoPose";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char robot_localization__srv__SetDatum_Event__TYPE_NAME[] = "robot_localization/srv/SetDatum_Event";
static char robot_localization__srv__SetDatum_Request__TYPE_NAME[] = "robot_localization/srv/SetDatum_Request";
static char robot_localization__srv__SetDatum_Response__TYPE_NAME[] = "robot_localization/srv/SetDatum_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robot_localization__srv__SetDatum__FIELD_NAME__request_message[] = "request_message";
static char robot_localization__srv__SetDatum__FIELD_NAME__response_message[] = "response_message";
static char robot_localization__srv__SetDatum__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetDatum__FIELDS[] = {
  {
    {robot_localization__srv__SetDatum__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_localization__srv__SetDatum_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_localization__srv__SetDatum_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robot_localization__srv__SetDatum_Event__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_localization__srv__SetDatum__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geographic_msgs__msg__GeoPoint__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geographic_msgs__msg__GeoPose__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum_Event__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_localization__srv__SetDatum__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetDatum__TYPE_NAME, 31, 31},
      {robot_localization__srv__SetDatum__FIELDS, 3, 3},
    },
    {robot_localization__srv__SetDatum__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geographic_msgs__msg__GeoPoint__EXPECTED_HASH, geographic_msgs__msg__GeoPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geographic_msgs__msg__GeoPoint__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geographic_msgs__msg__GeoPose__EXPECTED_HASH, geographic_msgs__msg__GeoPose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geographic_msgs__msg__GeoPose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = robot_localization__srv__SetDatum_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = robot_localization__srv__SetDatum_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = robot_localization__srv__SetDatum_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_localization__srv__SetDatum_Request__FIELD_NAME__geo_pose[] = "geo_pose";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetDatum_Request__FIELDS[] = {
  {
    {robot_localization__srv__SetDatum_Request__FIELD_NAME__geo_pose, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geographic_msgs__msg__GeoPose__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_localization__srv__SetDatum_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geographic_msgs__msg__GeoPoint__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geographic_msgs__msg__GeoPose__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_localization__srv__SetDatum_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetDatum_Request__TYPE_NAME, 39, 39},
      {robot_localization__srv__SetDatum_Request__FIELDS, 1, 1},
    },
    {robot_localization__srv__SetDatum_Request__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&geographic_msgs__msg__GeoPoint__EXPECTED_HASH, geographic_msgs__msg__GeoPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geographic_msgs__msg__GeoPoint__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geographic_msgs__msg__GeoPose__EXPECTED_HASH, geographic_msgs__msg__GeoPose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geographic_msgs__msg__GeoPose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_localization__srv__SetDatum_Response__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetDatum_Response__FIELDS[] = {
  {
    {robot_localization__srv__SetDatum_Response__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
robot_localization__srv__SetDatum_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetDatum_Response__TYPE_NAME, 40, 40},
      {robot_localization__srv__SetDatum_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robot_localization__srv__SetDatum_Event__FIELD_NAME__info[] = "info";
static char robot_localization__srv__SetDatum_Event__FIELD_NAME__request[] = "request";
static char robot_localization__srv__SetDatum_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robot_localization__srv__SetDatum_Event__FIELDS[] = {
  {
    {robot_localization__srv__SetDatum_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_localization__srv__SetDatum_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robot_localization__srv__SetDatum_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robot_localization__srv__SetDatum_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geographic_msgs__msg__GeoPoint__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geographic_msgs__msg__GeoPose__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {robot_localization__srv__SetDatum_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_localization__srv__SetDatum_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_localization__srv__SetDatum_Event__TYPE_NAME, 37, 37},
      {robot_localization__srv__SetDatum_Event__FIELDS, 3, 3},
    },
    {robot_localization__srv__SetDatum_Event__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geographic_msgs__msg__GeoPoint__EXPECTED_HASH, geographic_msgs__msg__GeoPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geographic_msgs__msg__GeoPoint__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geographic_msgs__msg__GeoPose__EXPECTED_HASH, geographic_msgs__msg__GeoPose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geographic_msgs__msg__GeoPose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = robot_localization__srv__SetDatum_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = robot_localization__srv__SetDatum_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "geographic_msgs/GeoPose geo_pose\n"
  "---";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetDatum__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetDatum__TYPE_NAME, 31, 31},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 37, 37},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetDatum_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetDatum_Request__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetDatum_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetDatum_Response__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robot_localization__srv__SetDatum_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_localization__srv__SetDatum_Event__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetDatum__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetDatum__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geographic_msgs__msg__GeoPoint__get_individual_type_description_source(NULL);
    sources[3] = *geographic_msgs__msg__GeoPose__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[5] = *robot_localization__srv__SetDatum_Event__get_individual_type_description_source(NULL);
    sources[6] = *robot_localization__srv__SetDatum_Request__get_individual_type_description_source(NULL);
    sources[7] = *robot_localization__srv__SetDatum_Response__get_individual_type_description_source(NULL);
    sources[8] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetDatum_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetDatum_Request__get_individual_type_description_source(NULL),
    sources[1] = *geographic_msgs__msg__GeoPoint__get_individual_type_description_source(NULL);
    sources[2] = *geographic_msgs__msg__GeoPose__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetDatum_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetDatum_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_localization__srv__SetDatum_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_localization__srv__SetDatum_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geographic_msgs__msg__GeoPoint__get_individual_type_description_source(NULL);
    sources[3] = *geographic_msgs__msg__GeoPose__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[5] = *robot_localization__srv__SetDatum_Request__get_individual_type_description_source(NULL);
    sources[6] = *robot_localization__srv__SetDatum_Response__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
