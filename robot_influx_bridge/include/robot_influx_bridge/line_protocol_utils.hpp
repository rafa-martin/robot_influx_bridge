// SPDX-License-Identifier: MIT

#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <cstdint>
#include <string>

namespace robot_influx_bridge::line_protocol
{

inline std::string escape_measurement(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (char character : value) {
    if (character == ' ' || character == ',') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

inline std::string escape_tag_value(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (char character : value) {
    if (character == ' ' || character == ',' || character == '=') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

inline std::string escape_field_key(const std::string & value)
{
  return escape_tag_value(value);
}

inline std::string escape_string_field_value(const std::string & value)
{
  std::string escaped = "\"";
  escaped.reserve(value.size() + 2U);
  for (char character : value) {
    if (character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  escaped.push_back('"');
  return escaped;
}

inline int64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL
       + static_cast<int64_t>(stamp.nanosec);
}

inline std::string resolve_measurement_name(const std::string & configured, const std::string & fallback)
{
  return configured.empty() ? fallback : configured;
}

}  // namespace robot_influx_bridge::line_protocol
