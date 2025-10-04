// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <rclcpp/serialized_message.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rclcpp {
class Node;
class Time;
}  // namespace rclcpp

namespace robot_influx_bridge {

/// Context passed to translators describing how the incoming message should be mapped
/// into InfluxDB line protocol.
struct TranslatorContext {
  std::string measurement;
  std::unordered_map<std::string, std::string> static_tags;
  std::unordered_map<std::string, std::string> custom_config;
};

/// Base class for translator plugins. Implementations convert ROS 2 messages into one
/// or more InfluxDB line protocol entries.
class TranslatorBase {
public:
  virtual ~TranslatorBase() = default;

  /// Returns the ROS 2 type string handled by the translator, e.g. "sensor_msgs/msg/Imu".
  virtual std::string type_name() const = 0;

  /// Whether this translator expects to receive messages via ROS 2 subscription.
  virtual bool requires_subscription() const { return true; }

  /// Called once after the translator instance is created to provide access to the
  /// owning node and the static mapping context.
  virtual void configure(rclcpp::Node & node, const TranslatorContext & context)
  {
    (void)node;
    (void)context;
  }

  /// If the translator should be triggered periodically instead of (or in addition to)
  /// reacting to incoming messages, return the desired timer period. Returning
  /// std::nullopt disables timer invocation.
  virtual std::optional<std::chrono::nanoseconds> get_timer_period(
    const TranslatorContext & context) const
  {
    (void)context;
    return std::nullopt;
  }

  /// Called on the timer created according to `get_timer_period`. The default
  /// implementation performs no work.
  virtual std::vector<std::string> on_timer(
    const TranslatorContext & context,
    const rclcpp::Time & now)
  {
    (void)context;
    (void)now;
    return {};
  }

  /// Convert a serialized message into a collection of line protocol strings.
  virtual std::vector<std::string> to_line_protocol(
      const rclcpp::SerializedMessage & msg,
      const TranslatorContext & context) = 0;
};

}  // namespace robot_influx_bridge
