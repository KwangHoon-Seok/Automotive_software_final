// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ad_msgs:msg/Mission.idl
// generated code does not contain a copyright notice
#include "ad_msgs/msg/detail/mission__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ad_msgs/msg/detail/mission__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace ad_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ad_msgs::msg::MissionObject &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ad_msgs::msg::MissionObject &);
size_t get_serialized_size(
  const ad_msgs::msg::MissionObject &,
  size_t current_alignment);
size_t
max_serialized_size_MissionObject(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ad_msgs


namespace ad_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ad_msgs
cdr_serialize(
  const ad_msgs::msg::Mission & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: objects
  {
    size_t size = ros_message.objects.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      ad_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.objects[i],
        cdr);
    }
  }
  // Member: road_condition
  cdr << ros_message.road_condition;
  // Member: road_slope
  cdr << ros_message.road_slope;
  // Member: speed_limit
  cdr << ros_message.speed_limit;
  // Member: parking
  cdr << (ros_message.parking ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ad_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ad_msgs::msg::Mission & ros_message)
{
  // Member: objects
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.objects.resize(size);
    for (size_t i = 0; i < size; i++) {
      ad_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.objects[i]);
    }
  }

  // Member: road_condition
  cdr >> ros_message.road_condition;

  // Member: road_slope
  cdr >> ros_message.road_slope;

  // Member: speed_limit
  cdr >> ros_message.speed_limit;

  // Member: parking
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.parking = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ad_msgs
get_serialized_size(
  const ad_msgs::msg::Mission & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: objects
  {
    size_t array_size = ros_message.objects.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ad_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.objects[index], current_alignment);
    }
  }
  // Member: road_condition
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.road_condition.size() + 1);
  // Member: road_slope
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.road_slope.size() + 1);
  // Member: speed_limit
  {
    size_t item_size = sizeof(ros_message.speed_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: parking
  {
    size_t item_size = sizeof(ros_message.parking);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ad_msgs
max_serialized_size_Mission(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: objects
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ad_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_MissionObject(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: road_condition
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: road_slope
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: speed_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: parking
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ad_msgs::msg::Mission;
    is_plain =
      (
      offsetof(DataType, parking) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Mission__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ad_msgs::msg::Mission *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Mission__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ad_msgs::msg::Mission *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Mission__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ad_msgs::msg::Mission *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Mission__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Mission(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Mission__callbacks = {
  "ad_msgs::msg",
  "Mission",
  _Mission__cdr_serialize,
  _Mission__cdr_deserialize,
  _Mission__get_serialized_size,
  _Mission__max_serialized_size
};

static rosidl_message_type_support_t _Mission__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Mission__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ad_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ad_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<ad_msgs::msg::Mission>()
{
  return &ad_msgs::msg::typesupport_fastrtps_cpp::_Mission__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ad_msgs, msg, Mission)() {
  return &ad_msgs::msg::typesupport_fastrtps_cpp::_Mission__handle;
}

#ifdef __cplusplus
}
#endif
