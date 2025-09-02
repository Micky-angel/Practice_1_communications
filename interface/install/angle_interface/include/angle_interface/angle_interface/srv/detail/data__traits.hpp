// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from angle_interface:srv/Data.idl
// generated code does not contain a copyright notice

#ifndef ANGLE_INTERFACE__SRV__DETAIL__DATA__TRAITS_HPP_
#define ANGLE_INTERFACE__SRV__DETAIL__DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "angle_interface/srv/detail/data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace angle_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Data_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: i1
  {
    out << "i1: ";
    rosidl_generator_traits::value_to_yaml(msg.i1, out);
    out << ", ";
  }

  // member: i2
  {
    out << "i2: ";
    rosidl_generator_traits::value_to_yaml(msg.i2, out);
    out << ", ";
  }

  // member: i3
  {
    out << "i3: ";
    rosidl_generator_traits::value_to_yaml(msg.i3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Data_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: i1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i1: ";
    rosidl_generator_traits::value_to_yaml(msg.i1, out);
    out << "\n";
  }

  // member: i2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i2: ";
    rosidl_generator_traits::value_to_yaml(msg.i2, out);
    out << "\n";
  }

  // member: i3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i3: ";
    rosidl_generator_traits::value_to_yaml(msg.i3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Data_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace angle_interface

namespace rosidl_generator_traits
{

[[deprecated("use angle_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const angle_interface::srv::Data_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  angle_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use angle_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const angle_interface::srv::Data_Request & msg)
{
  return angle_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<angle_interface::srv::Data_Request>()
{
  return "angle_interface::srv::Data_Request";
}

template<>
inline const char * name<angle_interface::srv::Data_Request>()
{
  return "angle_interface/srv/Data_Request";
}

template<>
struct has_fixed_size<angle_interface::srv::Data_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<angle_interface::srv::Data_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<angle_interface::srv::Data_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace angle_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Data_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ox
  {
    out << "ox: ";
    rosidl_generator_traits::value_to_yaml(msg.ox, out);
    out << ", ";
  }

  // member: oy
  {
    out << "oy: ";
    rosidl_generator_traits::value_to_yaml(msg.oy, out);
    out << ", ";
  }

  // member: oz
  {
    out << "oz: ";
    rosidl_generator_traits::value_to_yaml(msg.oz, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Data_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ox: ";
    rosidl_generator_traits::value_to_yaml(msg.ox, out);
    out << "\n";
  }

  // member: oy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "oy: ";
    rosidl_generator_traits::value_to_yaml(msg.oy, out);
    out << "\n";
  }

  // member: oz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "oz: ";
    rosidl_generator_traits::value_to_yaml(msg.oz, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Data_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace angle_interface

namespace rosidl_generator_traits
{

[[deprecated("use angle_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const angle_interface::srv::Data_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  angle_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use angle_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const angle_interface::srv::Data_Response & msg)
{
  return angle_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<angle_interface::srv::Data_Response>()
{
  return "angle_interface::srv::Data_Response";
}

template<>
inline const char * name<angle_interface::srv::Data_Response>()
{
  return "angle_interface/srv/Data_Response";
}

template<>
struct has_fixed_size<angle_interface::srv::Data_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<angle_interface::srv::Data_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<angle_interface::srv::Data_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<angle_interface::srv::Data>()
{
  return "angle_interface::srv::Data";
}

template<>
inline const char * name<angle_interface::srv::Data>()
{
  return "angle_interface/srv/Data";
}

template<>
struct has_fixed_size<angle_interface::srv::Data>
  : std::integral_constant<
    bool,
    has_fixed_size<angle_interface::srv::Data_Request>::value &&
    has_fixed_size<angle_interface::srv::Data_Response>::value
  >
{
};

template<>
struct has_bounded_size<angle_interface::srv::Data>
  : std::integral_constant<
    bool,
    has_bounded_size<angle_interface::srv::Data_Request>::value &&
    has_bounded_size<angle_interface::srv::Data_Response>::value
  >
{
};

template<>
struct is_service<angle_interface::srv::Data>
  : std::true_type
{
};

template<>
struct is_service_request<angle_interface::srv::Data_Request>
  : std::true_type
{
};

template<>
struct is_service_response<angle_interface::srv::Data_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ANGLE_INTERFACE__SRV__DETAIL__DATA__TRAITS_HPP_
