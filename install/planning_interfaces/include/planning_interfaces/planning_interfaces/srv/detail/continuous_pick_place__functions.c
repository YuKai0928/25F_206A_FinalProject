// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from planning_interfaces:srv/ContinuousPickPlace.idl
// generated code does not contain a copyright notice
#include "planning_interfaces/srv/detail/continuous_pick_place__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pick_scan_pose`
// Member `place_scan_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
planning_interfaces__srv__ContinuousPickPlace_Request__init(planning_interfaces__srv__ContinuousPickPlace_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pick_scan_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pick_scan_pose)) {
    planning_interfaces__srv__ContinuousPickPlace_Request__fini(msg);
    return false;
  }
  // place_scan_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->place_scan_pose)) {
    planning_interfaces__srv__ContinuousPickPlace_Request__fini(msg);
    return false;
  }
  // pick_distance
  // retreat_distance
  // place_distance
  return true;
}

void
planning_interfaces__srv__ContinuousPickPlace_Request__fini(planning_interfaces__srv__ContinuousPickPlace_Request * msg)
{
  if (!msg) {
    return;
  }
  // pick_scan_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pick_scan_pose);
  // place_scan_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->place_scan_pose);
  // pick_distance
  // retreat_distance
  // place_distance
}

bool
planning_interfaces__srv__ContinuousPickPlace_Request__are_equal(const planning_interfaces__srv__ContinuousPickPlace_Request * lhs, const planning_interfaces__srv__ContinuousPickPlace_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pick_scan_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pick_scan_pose), &(rhs->pick_scan_pose)))
  {
    return false;
  }
  // place_scan_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->place_scan_pose), &(rhs->place_scan_pose)))
  {
    return false;
  }
  // pick_distance
  if (lhs->pick_distance != rhs->pick_distance) {
    return false;
  }
  // retreat_distance
  if (lhs->retreat_distance != rhs->retreat_distance) {
    return false;
  }
  // place_distance
  if (lhs->place_distance != rhs->place_distance) {
    return false;
  }
  return true;
}

bool
planning_interfaces__srv__ContinuousPickPlace_Request__copy(
  const planning_interfaces__srv__ContinuousPickPlace_Request * input,
  planning_interfaces__srv__ContinuousPickPlace_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pick_scan_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pick_scan_pose), &(output->pick_scan_pose)))
  {
    return false;
  }
  // place_scan_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->place_scan_pose), &(output->place_scan_pose)))
  {
    return false;
  }
  // pick_distance
  output->pick_distance = input->pick_distance;
  // retreat_distance
  output->retreat_distance = input->retreat_distance;
  // place_distance
  output->place_distance = input->place_distance;
  return true;
}

planning_interfaces__srv__ContinuousPickPlace_Request *
planning_interfaces__srv__ContinuousPickPlace_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  planning_interfaces__srv__ContinuousPickPlace_Request * msg = (planning_interfaces__srv__ContinuousPickPlace_Request *)allocator.allocate(sizeof(planning_interfaces__srv__ContinuousPickPlace_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(planning_interfaces__srv__ContinuousPickPlace_Request));
  bool success = planning_interfaces__srv__ContinuousPickPlace_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
planning_interfaces__srv__ContinuousPickPlace_Request__destroy(planning_interfaces__srv__ContinuousPickPlace_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    planning_interfaces__srv__ContinuousPickPlace_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__init(planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  planning_interfaces__srv__ContinuousPickPlace_Request * data = NULL;

  if (size) {
    data = (planning_interfaces__srv__ContinuousPickPlace_Request *)allocator.zero_allocate(size, sizeof(planning_interfaces__srv__ContinuousPickPlace_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = planning_interfaces__srv__ContinuousPickPlace_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        planning_interfaces__srv__ContinuousPickPlace_Request__fini(&data[i - 1]);
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
planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__fini(planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * array)
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
      planning_interfaces__srv__ContinuousPickPlace_Request__fini(&array->data[i]);
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

planning_interfaces__srv__ContinuousPickPlace_Request__Sequence *
planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * array = (planning_interfaces__srv__ContinuousPickPlace_Request__Sequence *)allocator.allocate(sizeof(planning_interfaces__srv__ContinuousPickPlace_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__destroy(planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__are_equal(const planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * lhs, const planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!planning_interfaces__srv__ContinuousPickPlace_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
planning_interfaces__srv__ContinuousPickPlace_Request__Sequence__copy(
  const planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * input,
  planning_interfaces__srv__ContinuousPickPlace_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(planning_interfaces__srv__ContinuousPickPlace_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    planning_interfaces__srv__ContinuousPickPlace_Request * data =
      (planning_interfaces__srv__ContinuousPickPlace_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!planning_interfaces__srv__ContinuousPickPlace_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          planning_interfaces__srv__ContinuousPickPlace_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!planning_interfaces__srv__ContinuousPickPlace_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
planning_interfaces__srv__ContinuousPickPlace_Response__init(planning_interfaces__srv__ContinuousPickPlace_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    planning_interfaces__srv__ContinuousPickPlace_Response__fini(msg);
    return false;
  }
  // slides_picked
  return true;
}

void
planning_interfaces__srv__ContinuousPickPlace_Response__fini(planning_interfaces__srv__ContinuousPickPlace_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // slides_picked
}

bool
planning_interfaces__srv__ContinuousPickPlace_Response__are_equal(const planning_interfaces__srv__ContinuousPickPlace_Response * lhs, const planning_interfaces__srv__ContinuousPickPlace_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // slides_picked
  if (lhs->slides_picked != rhs->slides_picked) {
    return false;
  }
  return true;
}

bool
planning_interfaces__srv__ContinuousPickPlace_Response__copy(
  const planning_interfaces__srv__ContinuousPickPlace_Response * input,
  planning_interfaces__srv__ContinuousPickPlace_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // slides_picked
  output->slides_picked = input->slides_picked;
  return true;
}

planning_interfaces__srv__ContinuousPickPlace_Response *
planning_interfaces__srv__ContinuousPickPlace_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  planning_interfaces__srv__ContinuousPickPlace_Response * msg = (planning_interfaces__srv__ContinuousPickPlace_Response *)allocator.allocate(sizeof(planning_interfaces__srv__ContinuousPickPlace_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(planning_interfaces__srv__ContinuousPickPlace_Response));
  bool success = planning_interfaces__srv__ContinuousPickPlace_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
planning_interfaces__srv__ContinuousPickPlace_Response__destroy(planning_interfaces__srv__ContinuousPickPlace_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    planning_interfaces__srv__ContinuousPickPlace_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__init(planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  planning_interfaces__srv__ContinuousPickPlace_Response * data = NULL;

  if (size) {
    data = (planning_interfaces__srv__ContinuousPickPlace_Response *)allocator.zero_allocate(size, sizeof(planning_interfaces__srv__ContinuousPickPlace_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = planning_interfaces__srv__ContinuousPickPlace_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        planning_interfaces__srv__ContinuousPickPlace_Response__fini(&data[i - 1]);
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
planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__fini(planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * array)
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
      planning_interfaces__srv__ContinuousPickPlace_Response__fini(&array->data[i]);
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

planning_interfaces__srv__ContinuousPickPlace_Response__Sequence *
planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * array = (planning_interfaces__srv__ContinuousPickPlace_Response__Sequence *)allocator.allocate(sizeof(planning_interfaces__srv__ContinuousPickPlace_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__destroy(planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__are_equal(const planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * lhs, const planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!planning_interfaces__srv__ContinuousPickPlace_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
planning_interfaces__srv__ContinuousPickPlace_Response__Sequence__copy(
  const planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * input,
  planning_interfaces__srv__ContinuousPickPlace_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(planning_interfaces__srv__ContinuousPickPlace_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    planning_interfaces__srv__ContinuousPickPlace_Response * data =
      (planning_interfaces__srv__ContinuousPickPlace_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!planning_interfaces__srv__ContinuousPickPlace_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          planning_interfaces__srv__ContinuousPickPlace_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!planning_interfaces__srv__ContinuousPickPlace_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
