// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from servicios:srv/SaveRoute.idl
// generated code does not contain a copyright notice

#ifndef SERVICIOS__SRV__DETAIL__SAVE_ROUTE__BUILDER_HPP_
#define SERVICIOS__SRV__DETAIL__SAVE_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "servicios/srv/detail/save_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace servicios
{

namespace srv
{

namespace builder
{

class Init_SaveRoute_Request_file_path
{
public:
  Init_SaveRoute_Request_file_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::servicios::srv::SaveRoute_Request file_path(::servicios::srv::SaveRoute_Request::_file_path_type arg)
  {
    msg_.file_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servicios::srv::SaveRoute_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::servicios::srv::SaveRoute_Request>()
{
  return servicios::srv::builder::Init_SaveRoute_Request_file_path();
}

}  // namespace servicios


namespace servicios
{

namespace srv
{

namespace builder
{

class Init_SaveRoute_Response_result
{
public:
  Init_SaveRoute_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::servicios::srv::SaveRoute_Response result(::servicios::srv::SaveRoute_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servicios::srv::SaveRoute_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::servicios::srv::SaveRoute_Response>()
{
  return servicios::srv::builder::Init_SaveRoute_Response_result();
}

}  // namespace servicios

#endif  // SERVICIOS__SRV__DETAIL__SAVE_ROUTE__BUILDER_HPP_
