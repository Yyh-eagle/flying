// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from learning_interface:msg/MyState.idl
// generated code does not contain a copyright notice
#include "learning_interface/msg/detail/my_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
learning_interface__msg__MyState__init(learning_interface__msg__MyState * msg)
{
  if (!msg) {
    return false;
  }
  // a
  // b
  return true;
}

void
learning_interface__msg__MyState__fini(learning_interface__msg__MyState * msg)
{
  if (!msg) {
    return;
  }
  // a
  // b
}

bool
learning_interface__msg__MyState__are_equal(const learning_interface__msg__MyState * lhs, const learning_interface__msg__MyState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // a
  if (lhs->a != rhs->a) {
    return false;
  }
  // b
  if (lhs->b != rhs->b) {
    return false;
  }
  return true;
}

bool
learning_interface__msg__MyState__copy(
  const learning_interface__msg__MyState * input,
  learning_interface__msg__MyState * output)
{
  if (!input || !output) {
    return false;
  }
  // a
  output->a = input->a;
  // b
  output->b = input->b;
  return true;
}

learning_interface__msg__MyState *
learning_interface__msg__MyState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  learning_interface__msg__MyState * msg = (learning_interface__msg__MyState *)allocator.allocate(sizeof(learning_interface__msg__MyState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(learning_interface__msg__MyState));
  bool success = learning_interface__msg__MyState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
learning_interface__msg__MyState__destroy(learning_interface__msg__MyState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    learning_interface__msg__MyState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
learning_interface__msg__MyState__Sequence__init(learning_interface__msg__MyState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  learning_interface__msg__MyState * data = NULL;

  if (size) {
    data = (learning_interface__msg__MyState *)allocator.zero_allocate(size, sizeof(learning_interface__msg__MyState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = learning_interface__msg__MyState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        learning_interface__msg__MyState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
learning_interface__msg__MyState__Sequence__fini(learning_interface__msg__MyState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      learning_interface__msg__MyState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

learning_interface__msg__MyState__Sequence *
learning_interface__msg__MyState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  learning_interface__msg__MyState__Sequence * array = (learning_interface__msg__MyState__Sequence *)allocator.allocate(sizeof(learning_interface__msg__MyState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = learning_interface__msg__MyState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
learning_interface__msg__MyState__Sequence__destroy(learning_interface__msg__MyState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    learning_interface__msg__MyState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
learning_interface__msg__MyState__Sequence__are_equal(const learning_interface__msg__MyState__Sequence * lhs, const learning_interface__msg__MyState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!learning_interface__msg__MyState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
learning_interface__msg__MyState__Sequence__copy(
  const learning_interface__msg__MyState__Sequence * input,
  learning_interface__msg__MyState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(learning_interface__msg__MyState);
    learning_interface__msg__MyState * data =
      (learning_interface__msg__MyState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!learning_interface__msg__MyState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          learning_interface__msg__MyState__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!learning_interface__msg__MyState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
