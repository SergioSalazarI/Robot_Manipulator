// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from servicios:srv/ReproduceRoute.idl
// generated code does not contain a copyright notice

#ifndef SERVICIOS__SRV__DETAIL__REPRODUCE_ROUTE__BUILDER_HPP_
#define SERVICIOS__SRV__DETAIL__REPRODUCE_ROUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "servicios/srv/detail/reproduce_route__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace servicios
{

namespace srv
{

namespace builder
{

class Init_ReproduceRoute_Request_file_path
{
public:
  Init_ReproduceRoute_Request_file_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::servicios::srv::ReproduceRoute_Request file_path(::servicios::srv::ReproduceRoute_Request::_file_path_type arg)
  {
    msg_.file_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servicios::srv::ReproduceRoute_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::servicios::srv::ReproduceRoute_Request>()
{
  return servicios::srv::builder::Init_ReproduceRoute_Request_file_path();
}

}  // namespace servicios


namespace servicios
{

namespace srv
{

namespace builder
{

class Init_ReproduceRoute_Response_ruta
{
public:
  Init_ReproduceRoute_Response_ruta()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::servicios::srv::ReproduceRoute_Response ruta(::servicios::srv::ReproduceRoute_Response::_ruta_type arg)
  {
    msg_.ruta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servicios::srv::ReproduceRoute_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::servicios::srv::ReproduceRoute_Response>()
{
  return servicios::srv::builder::Init_ReproduceRoute_Response_ruta();
}

}  // namespace servicios

#endif  // SERVICIOS__SRV__DETAIL__REPRODUCE_ROUTE__BUILDER_HPP_
