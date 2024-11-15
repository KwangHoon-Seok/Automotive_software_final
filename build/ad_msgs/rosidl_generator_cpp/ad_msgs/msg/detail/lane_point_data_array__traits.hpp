// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ad_msgs:msg/LanePointDataArray.idl
// generated code does not contain a copyright notice

#ifndef AD_MSGS__MSG__DETAIL__LANE_POINT_DATA_ARRAY__TRAITS_HPP_
#define AD_MSGS__MSG__DETAIL__LANE_POINT_DATA_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ad_msgs/msg/detail/lane_point_data_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'lane'
#include "ad_msgs/msg/detail/lane_point_data__traits.hpp"

namespace ad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LanePointDataArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame_id
  {
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: lane
  {
    if (msg.lane.size() == 0) {
      out << "lane: []";
    } else {
      out << "lane: [";
      size_t pending_items = msg.lane.size();
      for (auto item : msg.lane) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LanePointDataArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: lane
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.lane.size() == 0) {
      out << "lane: []\n";
    } else {
      out << "lane:\n";
      for (auto item : msg.lane) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LanePointDataArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ad_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ad_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ad_msgs::msg::LanePointDataArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  ad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ad_msgs::msg::LanePointDataArray & msg)
{
  return ad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ad_msgs::msg::LanePointDataArray>()
{
  return "ad_msgs::msg::LanePointDataArray";
}

template<>
inline const char * name<ad_msgs::msg::LanePointDataArray>()
{
  return "ad_msgs/msg/LanePointDataArray";
}

template<>
struct has_fixed_size<ad_msgs::msg::LanePointDataArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ad_msgs::msg::LanePointDataArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ad_msgs::msg::LanePointDataArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AD_MSGS__MSG__DETAIL__LANE_POINT_DATA_ARRAY__TRAITS_HPP_