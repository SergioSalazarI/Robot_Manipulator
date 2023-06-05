// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from servicios:srv/SaveRoute.idl
// generated code does not contain a copyright notice

#ifndef SERVICIOS__SRV__DETAIL__SAVE_ROUTE__STRUCT_H_
#define SERVICIOS__SRV__DETAIL__SAVE_ROUTE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SaveRoute in the package servicios.
typedef struct servicios__srv__SaveRoute_Request
{
  rosidl_runtime_c__String file_path;
} servicios__srv__SaveRoute_Request;

// Struct for a sequence of servicios__srv__SaveRoute_Request.
typedef struct servicios__srv__SaveRoute_Request__Sequence
{
  servicios__srv__SaveRoute_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} servicios__srv__SaveRoute_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SaveRoute in the package servicios.
typedef struct servicios__srv__SaveRoute_Response
{
  rosidl_runtime_c__String result;
} servicios__srv__SaveRoute_Response;

// Struct for a sequence of servicios__srv__SaveRoute_Response.
typedef struct servicios__srv__SaveRoute_Response__Sequence
{
  servicios__srv__SaveRoute_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} servicios__srv__SaveRoute_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVICIOS__SRV__DETAIL__SAVE_ROUTE__STRUCT_H_
