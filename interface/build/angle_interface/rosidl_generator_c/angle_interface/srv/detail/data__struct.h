// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from angle_interface:srv/Data.idl
// generated code does not contain a copyright notice

#ifndef ANGLE_INTERFACE__SRV__DETAIL__DATA__STRUCT_H_
#define ANGLE_INTERFACE__SRV__DETAIL__DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Data in the package angle_interface.
typedef struct angle_interface__srv__Data_Request
{
  double i1;
  double i2;
  double i3;
} angle_interface__srv__Data_Request;

// Struct for a sequence of angle_interface__srv__Data_Request.
typedef struct angle_interface__srv__Data_Request__Sequence
{
  angle_interface__srv__Data_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} angle_interface__srv__Data_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Data in the package angle_interface.
typedef struct angle_interface__srv__Data_Response
{
  double ox;
  double oy;
  double oz;
} angle_interface__srv__Data_Response;

// Struct for a sequence of angle_interface__srv__Data_Response.
typedef struct angle_interface__srv__Data_Response__Sequence
{
  angle_interface__srv__Data_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} angle_interface__srv__Data_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ANGLE_INTERFACE__SRV__DETAIL__DATA__STRUCT_H_
