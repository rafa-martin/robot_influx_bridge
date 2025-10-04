// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <rclcpp/serialized_message.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace robot_influx_bridge {

/// Context passed to translators describing how the incoming message should be mapped
/// into InfluxDB line protocol.
struct TranslatorContext {
  std::string measurement;
  std::unordered_map<std::string, std::string> static_tags;
};

/// Base class for translator plugins. Implementations convert ROS 2 messages into one
/// or more InfluxDB line protocol entries.
class TranslatorBase {
public:
  virtual ~TranslatorBase() = default;

  /// Returns the ROS 2 type string handled by the translator, e.g. "sensor_msgs/msg/Imu".
  virtual std::string type_name() const = 0;

  /// Convert a serialized message into a collection of line protocol strings.
  virtual std::vector<std::string> to_line_protocol(
      const rclcpp::SerializedMessage & msg,
      const TranslatorContext & context) = 0;
};

}  // namespace robot_influx_bridge
