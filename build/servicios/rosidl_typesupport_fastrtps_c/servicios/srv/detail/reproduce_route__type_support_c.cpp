// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from servicios:srv/ReproduceRoute.idl
// generated code does not contain a copyright notice
#include "servicios/srv/detail/reproduce_route__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "servicios/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "servicios/srv/detail/reproduce_route__struct.h"
#include "servicios/srv/detail/reproduce_route__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // file_path
#include "rosidl_runtime_c/string_functions.h"  // file_path

// forward declare type support functions


using _ReproduceRoute_Request__ros_msg_type = servicios__srv__ReproduceRoute_Request;

static bool _ReproduceRoute_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ReproduceRoute_Request__ros_msg_type * ros_message = static_cast<const _ReproduceRoute_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: file_path
  {
    const rosidl_runtime_c__String * str = &ros_message->file_path;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _ReproduceRoute_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ReproduceRoute_Request__ros_msg_type * ros_message = static_cast<_ReproduceRoute_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: file_path
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->file_path.data) {
      rosidl_runtime_c__String__init(&ros_message->file_path);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->file_path,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'file_path'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servicios
size_t get_serialized_size_servicios__srv__ReproduceRoute_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ReproduceRoute_Request__ros_msg_type * ros_message = static_cast<const _ReproduceRoute_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name file_path
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->file_path.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _ReproduceRoute_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_servicios__srv__ReproduceRoute_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servicios
size_t max_serialized_size_servicios__srv__ReproduceRoute_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: file_path
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _ReproduceRoute_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_servicios__srv__ReproduceRoute_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ReproduceRoute_Request = {
  "servicios::srv",
  "ReproduceRoute_Request",
  _ReproduceRoute_Request__cdr_serialize,
  _ReproduceRoute_Request__cdr_deserialize,
  _ReproduceRoute_Request__get_serialized_size,
  _ReproduceRoute_Request__max_serialized_size
};

static rosidl_message_type_support_t _ReproduceRoute_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ReproduceRoute_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, servicios, srv, ReproduceRoute_Request)() {
  return &_ReproduceRoute_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "servicios/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "servicios/srv/detail/reproduce_route__struct.h"
// already included above
// #include "servicios/srv/detail/reproduce_route__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // ruta
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // ruta

// forward declare type support functions


using _ReproduceRoute_Response__ros_msg_type = servicios__srv__ReproduceRoute_Response;

static bool _ReproduceRoute_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ReproduceRoute_Response__ros_msg_type * ros_message = static_cast<const _ReproduceRoute_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: ruta
  {
    const rosidl_runtime_c__String * str = &ros_message->ruta;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _ReproduceRoute_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ReproduceRoute_Response__ros_msg_type * ros_message = static_cast<_ReproduceRoute_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: ruta
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->ruta.data) {
      rosidl_runtime_c__String__init(&ros_message->ruta);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->ruta,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'ruta'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servicios
size_t get_serialized_size_servicios__srv__ReproduceRoute_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ReproduceRoute_Response__ros_msg_type * ros_message = static_cast<const _ReproduceRoute_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ruta
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->ruta.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _ReproduceRoute_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_servicios__srv__ReproduceRoute_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servicios
size_t max_serialized_size_servicios__srv__ReproduceRoute_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: ruta
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _ReproduceRoute_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_servicios__srv__ReproduceRoute_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ReproduceRoute_Response = {
  "servicios::srv",
  "ReproduceRoute_Response",
  _ReproduceRoute_Response__cdr_serialize,
  _ReproduceRoute_Response__cdr_deserialize,
  _ReproduceRoute_Response__get_serialized_size,
  _ReproduceRoute_Response__max_serialized_size
};

static rosidl_message_type_support_t _ReproduceRoute_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ReproduceRoute_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, servicios, srv, ReproduceRoute_Response)() {
  return &_ReproduceRoute_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "servicios/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "servicios/srv/reproduce_route.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ReproduceRoute__callbacks = {
  "servicios::srv",
  "ReproduceRoute",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, servicios, srv, ReproduceRoute_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, servicios, srv, ReproduceRoute_Response)(),
};

static rosidl_service_type_support_t ReproduceRoute__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ReproduceRoute__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, servicios, srv, ReproduceRoute)() {
  return &ReproduceRoute__handle;
}

#if defined(__cplusplus)
}
#endif
