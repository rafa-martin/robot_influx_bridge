// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <pluginlib/class_loader.hpp>
#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/influx_writer.hpp>

#include <robot_influx_bridge/bridge_parameters.hpp>

using robot_influx_bridge::TranslatorBase;
using robot_influx_bridge::Context;

namespace robot_influx_bridge {

struct Mapping {
  std::string topic;
  std::string type;        // "pkg/msg/Type"
  Context ctx;
  std::shared_ptr<rclcpp::GenericSubscription> sub;
  std::shared_ptr<TranslatorBase> translator;
};

struct InfluxBridgeNode : public rclcpp::Node {
  InfluxBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::shared_ptr<influx_bridge::ParamListener> param_listener_;
  influx_bridge::Params params_;

  std::unique_ptr<pluginlib::ClassLoader<TranslatorBase>> loader_;
  std::unique_ptr<robot_influx_bridge::InfluxWriter> writer_;
  std::vector<Mapping> mappings_;
  rclcpp::TimerBase::SharedPtr timer_;

  void add_mapping_from_params(const std::string& id);
};

}  // namespace robot_influx_bridge
