// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from learning_interface:msg/ObjectPosition.idl
// generated code does not contain a copyright notice

#ifndef LEARNING_INTERFACE__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_
#define LEARNING_INTERFACE__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__learning_interface__msg__ObjectPosition __attribute__((deprecated))
#else
# define DEPRECATED__learning_interface__msg__ObjectPosition __declspec(deprecated)
#endif

namespace learning_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectPosition_
{
  using Type = ObjectPosition_<ContainerAllocator>;

  explicit ObjectPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 6>::iterator, int32_t>(this->x.begin(), this->x.end(), 0l);
      std::fill<typename std::array<int32_t, 6>::iterator, int32_t>(this->y.begin(), this->y.end(), 0l);
      std::fill<typename std::array<int32_t, 6>::iterator, int32_t>(this->z.begin(), this->z.end(), 0l);
      this->f = 0l;
      this->kind = 0l;
    }
  }

  explicit ObjectPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : x(_alloc),
    y(_alloc),
    z(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 6>::iterator, int32_t>(this->x.begin(), this->x.end(), 0l);
      std::fill<typename std::array<int32_t, 6>::iterator, int32_t>(this->y.begin(), this->y.end(), 0l);
      std::fill<typename std::array<int32_t, 6>::iterator, int32_t>(this->z.begin(), this->z.end(), 0l);
      this->f = 0l;
      this->kind = 0l;
    }
  }

  // field types and members
  using _x_type =
    std::array<int32_t, 6>;
  _x_type x;
  using _y_type =
    std::array<int32_t, 6>;
  _y_type y;
  using _z_type =
    std::array<int32_t, 6>;
  _z_type z;
  using _f_type =
    int32_t;
  _f_type f;
  using _kind_type =
    int32_t;
  _kind_type kind;

  // setters for named parameter idiom
  Type & set__x(
    const std::array<int32_t, 6> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::array<int32_t, 6> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const std::array<int32_t, 6> & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__f(
    const int32_t & _arg)
  {
    this->f = _arg;
    return *this;
  }
  Type & set__kind(
    const int32_t & _arg)
  {
    this->kind = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    learning_interface::msg::ObjectPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const learning_interface::msg::ObjectPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      learning_interface::msg::ObjectPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      learning_interface::msg::ObjectPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__learning_interface__msg__ObjectPosition
    std::shared_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__learning_interface__msg__ObjectPosition
    std::shared_ptr<learning_interface::msg::ObjectPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectPosition_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->f != other.f) {
      return false;
    }
    if (this->kind != other.kind) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectPosition_

// alias to use template instance with default allocator
using ObjectPosition =
  learning_interface::msg::ObjectPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace learning_interface

#endif  // LEARNING_INTERFACE__MSG__DETAIL__OBJECT_POSITION__STRUCT_HPP_
