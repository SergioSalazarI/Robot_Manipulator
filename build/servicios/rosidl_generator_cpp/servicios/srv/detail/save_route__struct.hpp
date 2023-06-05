// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from servicios:srv/SaveRoute.idl
// generated code does not contain a copyright notice

#ifndef SERVICIOS__SRV__DETAIL__SAVE_ROUTE__STRUCT_HPP_
#define SERVICIOS__SRV__DETAIL__SAVE_ROUTE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__servicios__srv__SaveRoute_Request __attribute__((deprecated))
#else
# define DEPRECATED__servicios__srv__SaveRoute_Request __declspec(deprecated)
#endif

namespace servicios
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveRoute_Request_
{
  using Type = SaveRoute_Request_<ContainerAllocator>;

  explicit SaveRoute_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_path = "";
    }
  }

  explicit SaveRoute_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : file_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_path = "";
    }
  }

  // field types and members
  using _file_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_path_type file_path;

  // setters for named parameter idiom
  Type & set__file_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    servicios::srv::SaveRoute_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const servicios::srv::SaveRoute_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      servicios::srv::SaveRoute_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      servicios::srv::SaveRoute_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__servicios__srv__SaveRoute_Request
    std::shared_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__servicios__srv__SaveRoute_Request
    std::shared_ptr<servicios::srv::SaveRoute_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveRoute_Request_ & other) const
  {
    if (this->file_path != other.file_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveRoute_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveRoute_Request_

// alias to use template instance with default allocator
using SaveRoute_Request =
  servicios::srv::SaveRoute_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace servicios


#ifndef _WIN32
# define DEPRECATED__servicios__srv__SaveRoute_Response __attribute__((deprecated))
#else
# define DEPRECATED__servicios__srv__SaveRoute_Response __declspec(deprecated)
#endif

namespace servicios
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveRoute_Response_
{
  using Type = SaveRoute_Response_<ContainerAllocator>;

  explicit SaveRoute_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
    }
  }

  explicit SaveRoute_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
    }
  }

  // field types and members
  using _result_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    servicios::srv::SaveRoute_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const servicios::srv::SaveRoute_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      servicios::srv::SaveRoute_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      servicios::srv::SaveRoute_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__servicios__srv__SaveRoute_Response
    std::shared_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__servicios__srv__SaveRoute_Response
    std::shared_ptr<servicios::srv::SaveRoute_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveRoute_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveRoute_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveRoute_Response_

// alias to use template instance with default allocator
using SaveRoute_Response =
  servicios::srv::SaveRoute_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace servicios

namespace servicios
{

namespace srv
{

struct SaveRoute
{
  using Request = servicios::srv::SaveRoute_Request;
  using Response = servicios::srv::SaveRoute_Response;
};

}  // namespace srv

}  // namespace servicios

#endif  // SERVICIOS__SRV__DETAIL__SAVE_ROUTE__STRUCT_HPP_
