// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from learning_interface:msg/MyState.idl
// generated code does not contain a copyright notice

#ifndef LEARNING_INTERFACE__MSG__DETAIL__MY_STATE__STRUCT_HPP_
#define LEARNING_INTERFACE__MSG__DETAIL__MY_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__learning_interface__msg__MyState __attribute__((deprecated))
#else
# define DEPRECATED__learning_interface__msg__MyState __declspec(deprecated)
#endif

namespace learning_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MyState_
{
  using Type = MyState_<ContainerAllocator>;

  explicit MyState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0l;
      this->b = 0l;
    }
  }

  explicit MyState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0l;
      this->b = 0l;
    }
  }

  // field types and members
  using _a_type =
    int32_t;
  _a_type a;
  using _b_type =
    int32_t;
  _b_type b;

  // setters for named parameter idiom
  Type & set__a(
    const int32_t & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const int32_t & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    learning_interface::msg::MyState_<ContainerAllocator> *;
  using ConstRawPtr =
    const learning_interface::msg::MyState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<learning_interface::msg::MyState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<learning_interface::msg::MyState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      learning_interface::msg::MyState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<learning_interface::msg::MyState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      learning_interface::msg::MyState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<learning_interface::msg::MyState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<learning_interface::msg::MyState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<learning_interface::msg::MyState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__learning_interface__msg__MyState
    std::shared_ptr<learning_interface::msg::MyState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__learning_interface__msg__MyState
    std::shared_ptr<learning_interface::msg::MyState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MyState_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const MyState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MyState_

// alias to use template instance with default allocator
using MyState =
  learning_interface::msg::MyState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace learning_interface

#endif  // LEARNING_INTERFACE__MSG__DETAIL__MY_STATE__STRUCT_HPP_
