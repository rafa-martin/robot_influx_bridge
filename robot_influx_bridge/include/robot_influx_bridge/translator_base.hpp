// Copyright 2025, Rafael Martin. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of Rafael Martin nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#pragma once
#ifndef ROBOT_INFLUX_BRIDGE__TRANSLATOR_BASE_HPP_
#define ROBOT_INFLUX_BRIDGE__TRANSLATOR_BASE_HPP_

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

#endif  // ROBOT_INFLUX_BRIDGE__TRANSLATOR_BASE_HPP_
