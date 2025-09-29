// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <rclcpp/serialized_message.hpp>
#include <string>
#include <unordered_map>

namespace robot_influx_bridge {

struct Context {
  // static config for this mapping
  std::string measurement;
  std::unordered_map<std::string,std::string> static_tags; // tag_key->value
};

class TranslatorBase {
public:
  virtual ~TranslatorBase() = default;
  // ROS 2 type string, e.g. "sensor_msgs/msg/Imu"
  virtual std::string type_name() const = 0;
  // Build one or more line protocol lines from a serialized message.
  virtual std::vector<std::string> to_line_protocol(
      const rclcpp::SerializedMessage & msg,
      const Context & ctx) = 0;
};

} // namespace robot_influx_bridge
