// SPDX-License-Identifier: MIT

#ifndef ROBOT_INFLUX_BRIDGE__TRANSLATOR_UTILS_HPP_
#define ROBOT_INFLUX_BRIDGE__TRANSLATOR_UTILS_HPP_

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/line_protocol_utils.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_bridge
{

std::string build_series_name(
  const TranslatorContext & context,
  const std::string & default_measurement);

void append_timestamp_if_valid(std::ostringstream & stream, const builtin_interfaces::msg::Time & stamp);

}  // namespace robot_influx_bridge

#endif  // ROBOT_INFLUX_BRIDGE__TRANSLATOR_UTILS_HPP_
