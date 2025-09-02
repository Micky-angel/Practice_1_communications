// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from angle_interface:srv/Data.idl
// generated code does not contain a copyright notice

#ifndef ANGLE_INTERFACE__SRV__DETAIL__DATA__STRUCT_HPP_
#define ANGLE_INTERFACE__SRV__DETAIL__DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__angle_interface__srv__Data_Request __attribute__((deprecated))
#else
# define DEPRECATED__angle_interface__srv__Data_Request __declspec(deprecated)
#endif

namespace angle_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Data_Request_
{
  using Type = Data_Request_<ContainerAllocator>;

  explicit Data_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->i1 = 0.0;
      this->i2 = 0.0;
      this->i3 = 0.0;
    }
  }

  explicit Data_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->i1 = 0.0;
      this->i2 = 0.0;
      this->i3 = 0.0;
    }
  }

  // field types and members
  using _i1_type =
    double;
  _i1_type i1;
  using _i2_type =
    double;
  _i2_type i2;
  using _i3_type =
    double;
  _i3_type i3;

  // setters for named parameter idiom
  Type & set__i1(
    const double & _arg)
  {
    this->i1 = _arg;
    return *this;
  }
  Type & set__i2(
    const double & _arg)
  {
    this->i2 = _arg;
    return *this;
  }
  Type & set__i3(
    const double & _arg)
  {
    this->i3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    angle_interface::srv::Data_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const angle_interface::srv::Data_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<angle_interface::srv::Data_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<angle_interface::srv::Data_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      angle_interface::srv::Data_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<angle_interface::srv::Data_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      angle_interface::srv::Data_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<angle_interface::srv::Data_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<angle_interface::srv::Data_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<angle_interface::srv::Data_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__angle_interface__srv__Data_Request
    std::shared_ptr<angle_interface::srv::Data_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__angle_interface__srv__Data_Request
    std::shared_ptr<angle_interface::srv::Data_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Data_Request_ & other) const
  {
    if (this->i1 != other.i1) {
      return false;
    }
    if (this->i2 != other.i2) {
      return false;
    }
    if (this->i3 != other.i3) {
      return false;
    }
    return true;
  }
  bool operator!=(const Data_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Data_Request_

// alias to use template instance with default allocator
using Data_Request =
  angle_interface::srv::Data_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace angle_interface


#ifndef _WIN32
# define DEPRECATED__angle_interface__srv__Data_Response __attribute__((deprecated))
#else
# define DEPRECATED__angle_interface__srv__Data_Response __declspec(deprecated)
#endif

namespace angle_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Data_Response_
{
  using Type = Data_Response_<ContainerAllocator>;

  explicit Data_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ox = 0.0;
      this->oy = 0.0;
      this->oz = 0.0;
    }
  }

  explicit Data_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ox = 0.0;
      this->oy = 0.0;
      this->oz = 0.0;
    }
  }

  // field types and members
  using _ox_type =
    double;
  _ox_type ox;
  using _oy_type =
    double;
  _oy_type oy;
  using _oz_type =
    double;
  _oz_type oz;

  // setters for named parameter idiom
  Type & set__ox(
    const double & _arg)
  {
    this->ox = _arg;
    return *this;
  }
  Type & set__oy(
    const double & _arg)
  {
    this->oy = _arg;
    return *this;
  }
  Type & set__oz(
    const double & _arg)
  {
    this->oz = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    angle_interface::srv::Data_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const angle_interface::srv::Data_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<angle_interface::srv::Data_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<angle_interface::srv::Data_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      angle_interface::srv::Data_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<angle_interface::srv::Data_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      angle_interface::srv::Data_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<angle_interface::srv::Data_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<angle_interface::srv::Data_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<angle_interface::srv::Data_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__angle_interface__srv__Data_Response
    std::shared_ptr<angle_interface::srv::Data_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__angle_interface__srv__Data_Response
    std::shared_ptr<angle_interface::srv::Data_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Data_Response_ & other) const
  {
    if (this->ox != other.ox) {
      return false;
    }
    if (this->oy != other.oy) {
      return false;
    }
    if (this->oz != other.oz) {
      return false;
    }
    return true;
  }
  bool operator!=(const Data_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Data_Response_

// alias to use template instance with default allocator
using Data_Response =
  angle_interface::srv::Data_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace angle_interface

namespace angle_interface
{

namespace srv
{

struct Data
{
  using Request = angle_interface::srv::Data_Request;
  using Response = angle_interface::srv::Data_Response;
};

}  // namespace srv

}  // namespace angle_interface

#endif  // ANGLE_INTERFACE__SRV__DETAIL__DATA__STRUCT_HPP_
