// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from angle_interface:srv/Data.idl
// generated code does not contain a copyright notice
#include "angle_interface/srv/detail/data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
angle_interface__srv__Data_Request__init(angle_interface__srv__Data_Request * msg)
{
  if (!msg) {
    return false;
  }
  // i1
  // i2
  // i3
  return true;
}

void
angle_interface__srv__Data_Request__fini(angle_interface__srv__Data_Request * msg)
{
  if (!msg) {
    return;
  }
  // i1
  // i2
  // i3
}

bool
angle_interface__srv__Data_Request__are_equal(const angle_interface__srv__Data_Request * lhs, const angle_interface__srv__Data_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // i1
  if (lhs->i1 != rhs->i1) {
    return false;
  }
  // i2
  if (lhs->i2 != rhs->i2) {
    return false;
  }
  // i3
  if (lhs->i3 != rhs->i3) {
    return false;
  }
  return true;
}

bool
angle_interface__srv__Data_Request__copy(
  const angle_interface__srv__Data_Request * input,
  angle_interface__srv__Data_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // i1
  output->i1 = input->i1;
  // i2
  output->i2 = input->i2;
  // i3
  output->i3 = input->i3;
  return true;
}

angle_interface__srv__Data_Request *
angle_interface__srv__Data_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  angle_interface__srv__Data_Request * msg = (angle_interface__srv__Data_Request *)allocator.allocate(sizeof(angle_interface__srv__Data_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(angle_interface__srv__Data_Request));
  bool success = angle_interface__srv__Data_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
angle_interface__srv__Data_Request__destroy(angle_interface__srv__Data_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    angle_interface__srv__Data_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
angle_interface__srv__Data_Request__Sequence__init(angle_interface__srv__Data_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  angle_interface__srv__Data_Request * data = NULL;

  if (size) {
    data = (angle_interface__srv__Data_Request *)allocator.zero_allocate(size, sizeof(angle_interface__srv__Data_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = angle_interface__srv__Data_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        angle_interface__srv__Data_Request__fini(&data[i - 1]);
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
angle_interface__srv__Data_Request__Sequence__fini(angle_interface__srv__Data_Request__Sequence * array)
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
      angle_interface__srv__Data_Request__fini(&array->data[i]);
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

angle_interface__srv__Data_Request__Sequence *
angle_interface__srv__Data_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  angle_interface__srv__Data_Request__Sequence * array = (angle_interface__srv__Data_Request__Sequence *)allocator.allocate(sizeof(angle_interface__srv__Data_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = angle_interface__srv__Data_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
angle_interface__srv__Data_Request__Sequence__destroy(angle_interface__srv__Data_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    angle_interface__srv__Data_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
angle_interface__srv__Data_Request__Sequence__are_equal(const angle_interface__srv__Data_Request__Sequence * lhs, const angle_interface__srv__Data_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!angle_interface__srv__Data_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
angle_interface__srv__Data_Request__Sequence__copy(
  const angle_interface__srv__Data_Request__Sequence * input,
  angle_interface__srv__Data_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(angle_interface__srv__Data_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    angle_interface__srv__Data_Request * data =
      (angle_interface__srv__Data_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!angle_interface__srv__Data_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          angle_interface__srv__Data_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!angle_interface__srv__Data_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
angle_interface__srv__Data_Response__init(angle_interface__srv__Data_Response * msg)
{
  if (!msg) {
    return false;
  }
  // ox
  // oy
  // oz
  return true;
}

void
angle_interface__srv__Data_Response__fini(angle_interface__srv__Data_Response * msg)
{
  if (!msg) {
    return;
  }
  // ox
  // oy
  // oz
}

bool
angle_interface__srv__Data_Response__are_equal(const angle_interface__srv__Data_Response * lhs, const angle_interface__srv__Data_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ox
  if (lhs->ox != rhs->ox) {
    return false;
  }
  // oy
  if (lhs->oy != rhs->oy) {
    return false;
  }
  // oz
  if (lhs->oz != rhs->oz) {
    return false;
  }
  return true;
}

bool
angle_interface__srv__Data_Response__copy(
  const angle_interface__srv__Data_Response * input,
  angle_interface__srv__Data_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // ox
  output->ox = input->ox;
  // oy
  output->oy = input->oy;
  // oz
  output->oz = input->oz;
  return true;
}

angle_interface__srv__Data_Response *
angle_interface__srv__Data_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  angle_interface__srv__Data_Response * msg = (angle_interface__srv__Data_Response *)allocator.allocate(sizeof(angle_interface__srv__Data_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(angle_interface__srv__Data_Response));
  bool success = angle_interface__srv__Data_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
angle_interface__srv__Data_Response__destroy(angle_interface__srv__Data_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    angle_interface__srv__Data_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
angle_interface__srv__Data_Response__Sequence__init(angle_interface__srv__Data_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  angle_interface__srv__Data_Response * data = NULL;

  if (size) {
    data = (angle_interface__srv__Data_Response *)allocator.zero_allocate(size, sizeof(angle_interface__srv__Data_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = angle_interface__srv__Data_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        angle_interface__srv__Data_Response__fini(&data[i - 1]);
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
angle_interface__srv__Data_Response__Sequence__fini(angle_interface__srv__Data_Response__Sequence * array)
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
      angle_interface__srv__Data_Response__fini(&array->data[i]);
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

angle_interface__srv__Data_Response__Sequence *
angle_interface__srv__Data_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  angle_interface__srv__Data_Response__Sequence * array = (angle_interface__srv__Data_Response__Sequence *)allocator.allocate(sizeof(angle_interface__srv__Data_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = angle_interface__srv__Data_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
angle_interface__srv__Data_Response__Sequence__destroy(angle_interface__srv__Data_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    angle_interface__srv__Data_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
angle_interface__srv__Data_Response__Sequence__are_equal(const angle_interface__srv__Data_Response__Sequence * lhs, const angle_interface__srv__Data_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!angle_interface__srv__Data_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
angle_interface__srv__Data_Response__Sequence__copy(
  const angle_interface__srv__Data_Response__Sequence * input,
  angle_interface__srv__Data_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(angle_interface__srv__Data_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    angle_interface__srv__Data_Response * data =
      (angle_interface__srv__Data_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!angle_interface__srv__Data_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          angle_interface__srv__Data_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!angle_interface__srv__Data_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
