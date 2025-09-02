// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from angle_interface:srv/Data.idl
// generated code does not contain a copyright notice
#include "angle_interface/srv/detail/data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "angle_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "angle_interface/srv/detail/data__struct.h"
#include "angle_interface/srv/detail/data__functions.h"
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


// forward declare type support functions


using _Data_Request__ros_msg_type = angle_interface__srv__Data_Request;

static bool _Data_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Data_Request__ros_msg_type * ros_message = static_cast<const _Data_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: i1
  {
    cdr << ros_message->i1;
  }

  // Field name: i2
  {
    cdr << ros_message->i2;
  }

  // Field name: i3
  {
    cdr << ros_message->i3;
  }

  return true;
}

static bool _Data_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Data_Request__ros_msg_type * ros_message = static_cast<_Data_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: i1
  {
    cdr >> ros_message->i1;
  }

  // Field name: i2
  {
    cdr >> ros_message->i2;
  }

  // Field name: i3
  {
    cdr >> ros_message->i3;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_angle_interface
size_t get_serialized_size_angle_interface__srv__Data_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Data_Request__ros_msg_type * ros_message = static_cast<const _Data_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name i1
  {
    size_t item_size = sizeof(ros_message->i1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name i2
  {
    size_t item_size = sizeof(ros_message->i2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name i3
  {
    size_t item_size = sizeof(ros_message->i3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Data_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_angle_interface__srv__Data_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_angle_interface
size_t max_serialized_size_angle_interface__srv__Data_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: i1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: i2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: i3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = angle_interface__srv__Data_Request;
    is_plain =
      (
      offsetof(DataType, i3) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Data_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_angle_interface__srv__Data_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Data_Request = {
  "angle_interface::srv",
  "Data_Request",
  _Data_Request__cdr_serialize,
  _Data_Request__cdr_deserialize,
  _Data_Request__get_serialized_size,
  _Data_Request__max_serialized_size
};

static rosidl_message_type_support_t _Data_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Data_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, angle_interface, srv, Data_Request)() {
  return &_Data_Request__type_support;
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
// #include "angle_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "angle_interface/srv/detail/data__struct.h"
// already included above
// #include "angle_interface/srv/detail/data__functions.h"
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


// forward declare type support functions


using _Data_Response__ros_msg_type = angle_interface__srv__Data_Response;

static bool _Data_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Data_Response__ros_msg_type * ros_message = static_cast<const _Data_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: ox
  {
    cdr << ros_message->ox;
  }

  // Field name: oy
  {
    cdr << ros_message->oy;
  }

  // Field name: oz
  {
    cdr << ros_message->oz;
  }

  return true;
}

static bool _Data_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Data_Response__ros_msg_type * ros_message = static_cast<_Data_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: ox
  {
    cdr >> ros_message->ox;
  }

  // Field name: oy
  {
    cdr >> ros_message->oy;
  }

  // Field name: oz
  {
    cdr >> ros_message->oz;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_angle_interface
size_t get_serialized_size_angle_interface__srv__Data_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Data_Response__ros_msg_type * ros_message = static_cast<const _Data_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ox
  {
    size_t item_size = sizeof(ros_message->ox);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name oy
  {
    size_t item_size = sizeof(ros_message->oy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name oz
  {
    size_t item_size = sizeof(ros_message->oz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Data_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_angle_interface__srv__Data_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_angle_interface
size_t max_serialized_size_angle_interface__srv__Data_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: ox
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: oy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: oz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = angle_interface__srv__Data_Response;
    is_plain =
      (
      offsetof(DataType, oz) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Data_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_angle_interface__srv__Data_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Data_Response = {
  "angle_interface::srv",
  "Data_Response",
  _Data_Response__cdr_serialize,
  _Data_Response__cdr_deserialize,
  _Data_Response__get_serialized_size,
  _Data_Response__max_serialized_size
};

static rosidl_message_type_support_t _Data_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Data_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, angle_interface, srv, Data_Response)() {
  return &_Data_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "angle_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "angle_interface/srv/data.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Data__callbacks = {
  "angle_interface::srv",
  "Data",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, angle_interface, srv, Data_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, angle_interface, srv, Data_Response)(),
};

static rosidl_service_type_support_t Data__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Data__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, angle_interface, srv, Data)() {
  return &Data__handle;
}

#if defined(__cplusplus)
}
#endif
