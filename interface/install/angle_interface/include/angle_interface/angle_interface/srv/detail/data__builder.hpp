// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from angle_interface:srv/Data.idl
// generated code does not contain a copyright notice

#ifndef ANGLE_INTERFACE__SRV__DETAIL__DATA__BUILDER_HPP_
#define ANGLE_INTERFACE__SRV__DETAIL__DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "angle_interface/srv/detail/data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace angle_interface
{

namespace srv
{

namespace builder
{

class Init_Data_Request_i3
{
public:
  explicit Init_Data_Request_i3(::angle_interface::srv::Data_Request & msg)
  : msg_(msg)
  {}
  ::angle_interface::srv::Data_Request i3(::angle_interface::srv::Data_Request::_i3_type arg)
  {
    msg_.i3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::angle_interface::srv::Data_Request msg_;
};

class Init_Data_Request_i2
{
public:
  explicit Init_Data_Request_i2(::angle_interface::srv::Data_Request & msg)
  : msg_(msg)
  {}
  Init_Data_Request_i3 i2(::angle_interface::srv::Data_Request::_i2_type arg)
  {
    msg_.i2 = std::move(arg);
    return Init_Data_Request_i3(msg_);
  }

private:
  ::angle_interface::srv::Data_Request msg_;
};

class Init_Data_Request_i1
{
public:
  Init_Data_Request_i1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Data_Request_i2 i1(::angle_interface::srv::Data_Request::_i1_type arg)
  {
    msg_.i1 = std::move(arg);
    return Init_Data_Request_i2(msg_);
  }

private:
  ::angle_interface::srv::Data_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::angle_interface::srv::Data_Request>()
{
  return angle_interface::srv::builder::Init_Data_Request_i1();
}

}  // namespace angle_interface


namespace angle_interface
{

namespace srv
{

namespace builder
{

class Init_Data_Response_oz
{
public:
  explicit Init_Data_Response_oz(::angle_interface::srv::Data_Response & msg)
  : msg_(msg)
  {}
  ::angle_interface::srv::Data_Response oz(::angle_interface::srv::Data_Response::_oz_type arg)
  {
    msg_.oz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::angle_interface::srv::Data_Response msg_;
};

class Init_Data_Response_oy
{
public:
  explicit Init_Data_Response_oy(::angle_interface::srv::Data_Response & msg)
  : msg_(msg)
  {}
  Init_Data_Response_oz oy(::angle_interface::srv::Data_Response::_oy_type arg)
  {
    msg_.oy = std::move(arg);
    return Init_Data_Response_oz(msg_);
  }

private:
  ::angle_interface::srv::Data_Response msg_;
};

class Init_Data_Response_ox
{
public:
  Init_Data_Response_ox()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Data_Response_oy ox(::angle_interface::srv::Data_Response::_ox_type arg)
  {
    msg_.ox = std::move(arg);
    return Init_Data_Response_oy(msg_);
  }

private:
  ::angle_interface::srv::Data_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::angle_interface::srv::Data_Response>()
{
  return angle_interface::srv::builder::Init_Data_Response_ox();
}

}  // namespace angle_interface

#endif  // ANGLE_INTERFACE__SRV__DETAIL__DATA__BUILDER_HPP_
