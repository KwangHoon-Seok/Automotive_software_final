// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ad_msgs:msg/Mission.idl
// generated code does not contain a copyright notice

#ifndef AD_MSGS__MSG__DETAIL__MISSION__FUNCTIONS_H_
#define AD_MSGS__MSG__DETAIL__MISSION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ad_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ad_msgs/msg/detail/mission__struct.h"

/// Initialize msg/Mission message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ad_msgs__msg__Mission
 * )) before or use
 * ad_msgs__msg__Mission__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
bool
ad_msgs__msg__Mission__init(ad_msgs__msg__Mission * msg);

/// Finalize msg/Mission message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
void
ad_msgs__msg__Mission__fini(ad_msgs__msg__Mission * msg);

/// Create msg/Mission message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ad_msgs__msg__Mission__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
ad_msgs__msg__Mission *
ad_msgs__msg__Mission__create();

/// Destroy msg/Mission message.
/**
 * It calls
 * ad_msgs__msg__Mission__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
void
ad_msgs__msg__Mission__destroy(ad_msgs__msg__Mission * msg);

/// Check for msg/Mission message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
bool
ad_msgs__msg__Mission__are_equal(const ad_msgs__msg__Mission * lhs, const ad_msgs__msg__Mission * rhs);

/// Copy a msg/Mission message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
bool
ad_msgs__msg__Mission__copy(
  const ad_msgs__msg__Mission * input,
  ad_msgs__msg__Mission * output);

/// Initialize array of msg/Mission messages.
/**
 * It allocates the memory for the number of elements and calls
 * ad_msgs__msg__Mission__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
bool
ad_msgs__msg__Mission__Sequence__init(ad_msgs__msg__Mission__Sequence * array, size_t size);

/// Finalize array of msg/Mission messages.
/**
 * It calls
 * ad_msgs__msg__Mission__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
void
ad_msgs__msg__Mission__Sequence__fini(ad_msgs__msg__Mission__Sequence * array);

/// Create array of msg/Mission messages.
/**
 * It allocates the memory for the array and calls
 * ad_msgs__msg__Mission__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
ad_msgs__msg__Mission__Sequence *
ad_msgs__msg__Mission__Sequence__create(size_t size);

/// Destroy array of msg/Mission messages.
/**
 * It calls
 * ad_msgs__msg__Mission__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
void
ad_msgs__msg__Mission__Sequence__destroy(ad_msgs__msg__Mission__Sequence * array);

/// Check for msg/Mission message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
bool
ad_msgs__msg__Mission__Sequence__are_equal(const ad_msgs__msg__Mission__Sequence * lhs, const ad_msgs__msg__Mission__Sequence * rhs);

/// Copy an array of msg/Mission messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ad_msgs
bool
ad_msgs__msg__Mission__Sequence__copy(
  const ad_msgs__msg__Mission__Sequence * input,
  ad_msgs__msg__Mission__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AD_MSGS__MSG__DETAIL__MISSION__FUNCTIONS_H_