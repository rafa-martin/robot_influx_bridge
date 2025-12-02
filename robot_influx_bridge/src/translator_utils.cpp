// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_utils.hpp>

#include <algorithm>
#include <utility>
#include <vector>

namespace robot_influx_bridge
{

std::string build_series_name(
  const TranslatorContext & context,
  const std::string & default_measurement)
{
  const std::string measurement =
    line_protocol::resolve_measurement_name(context.measurement, default_measurement);

  std::ostringstream series;
  series << line_protocol::escape_measurement(measurement);

  std::vector<std::pair<std::string, std::string>> tags{
    context.static_tags.begin(), context.static_tags.end()};
  std::sort(tags.begin(), tags.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.first < rhs.first;
  });

  for (const auto & [key, value] : tags) {
    series << ',' << line_protocol::escape_field_key(key) << '='
           << line_protocol::escape_tag_value(value);
  }

  return series.str();
}

void append_timestamp_if_valid(std::ostringstream & stream, const builtin_interfaces::msg::Time & stamp)
{
  const int64_t time_ns = line_protocol::stamp_to_nanoseconds(stamp);
  if (time_ns != 0) {
    stream << ' ' << time_ns;
  }
}

}  // namespace robot_influx_bridge
