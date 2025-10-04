// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/time.hpp>
#include <pluginlib/class_loader.hpp>

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/influx_writer.hpp>
#include <robot_influx_bridge/bridge_parameters.hpp>

#include <memory>
#include <string>
#include <vector>

namespace robot_influx_bridge {

/// Stores the objects required to service a single mapping between a ROS topic and
/// an InfluxDB measurement.
struct TopicMapping {
  std::string topic_name;
  TranslatorContext translator_context;
  std::shared_ptr<rclcpp::GenericSubscription> subscription;
  std::shared_ptr<TranslatorBase> translator;

  struct DownsampleState {
    rclcpp::Duration min_gap{0, 0};
    rclcpp::Time last_emit;
    bool has_last_emit{false};
  };

  std::shared_ptr<DownsampleState> downsample_state;
};

/// Node responsible for loading translator plugins and forwarding telemetry batches to
/// InfluxDB.
struct InfluxBridgeNode : public rclcpp::Node {
  explicit InfluxBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::shared_ptr<influx_bridge::ParamListener> param_listener_;
  influx_bridge::Params params_;

  std::unique_ptr<pluginlib::ClassLoader<TranslatorBase>> translator_loader_;
  std::unique_ptr<robot_influx_bridge::InfluxWriter> writer_;
  std::vector<TopicMapping> topic_mappings_;

  void addMappingFromParams(const std::string& mapping_id);
};

}  // namespace robot_influx_bridge
