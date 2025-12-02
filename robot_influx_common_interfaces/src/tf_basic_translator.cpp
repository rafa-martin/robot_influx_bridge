// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>
#include <robot_influx_bridge/line_protocol_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robot_influx_common_interfaces
{

class TfBasicTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "geometry_msgs/msg/TransformStamped"; }

  bool requires_subscription() const override { return false; }

  void configure(rclcpp::Node & node, const robot_influx_bridge::TranslatorContext & context) override
  {
    (void)context;
    logger_ = node.get_logger().get_child("tf_basic_translator");
    clock_ = node.get_clock();

    const auto maybe_source = context.custom_config.find("source_frame");
    if (maybe_source != context.custom_config.end()) {
      source_frame_ = maybe_source->second;
    }
    const auto maybe_target = context.custom_config.find("target_frame");
    if (maybe_target != context.custom_config.end()) {
      target_frame_ = maybe_target->second;
    }
    const auto maybe_parent = context.custom_config.find("parent_frame");
    if (maybe_parent != context.custom_config.end()) {
      parent_frame_ = maybe_parent->second;
    }

    const double buffer_time = parse_double(context.custom_config, "buffer_time_sec", 10.0);
    lookup_timeout_ = tf2::durationFromSec(parse_double(context.custom_config, "lookup_timeout_sec", 0.1));
    update_period_ = std::chrono::nanoseconds(parse_period(context.custom_config));

    buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock(), tf2::durationFromSec(buffer_time));

    try {
      auto node_base = node.get_node_base_interface();
      auto node_clock = node.get_node_clock_interface();
      auto node_logging = node.get_node_logging_interface();
      auto node_timers = node.get_node_timers_interface();

      if (!node_base || !node_clock || !node_logging || !node_timers) {
        RCLCPP_ERROR(logger_,
          "Unable to create TransformListener: missing required node interfaces (base:%s clock:%s logging:%s timers:%s)",
          node_base ? "ok" : "missing",
          node_clock ? "ok" : "missing",
          node_logging ? "ok" : "missing",
          node_timers ? "ok" : "missing");
      } else {
        listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(logger_, "Unable to create TransformListener: %s", ex.what());
    }

    configured_ = buffer_ != nullptr && listener_ != nullptr;
    if (!configured_) {
      RCLCPP_WARN(logger_,
        "TfBasicTranslator failed to initialize TF listener. Periodic lookups will be disabled.");
    }
  }

  std::optional<std::chrono::nanoseconds> get_timer_period(
    const robot_influx_bridge::TranslatorContext & context) const override
  {
    (void)context;
    if (!configured_) {
      return std::nullopt;
    }
    return update_period_;
  }

  std::vector<std::string> on_timer(
    const robot_influx_bridge::TranslatorContext & context,
    const rclcpp::Time &)
    override
  {
    if (!configured_) {
      return {};
    }
    if (!buffer_ || !listener_) {
      if (clock_) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
          "TfBasicTranslator is not fully initialized. Skipping lookup.");
      }
      return {};
    }
    if (source_frame_.empty() || target_frame_.empty()) {
      if (clock_) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
          "TfBasicTranslator requires both source_frame and target_frame to be configured");
      }
      return {};
    }

    try {
      const auto transform = buffer_->lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero, lookup_timeout_);

      const int64_t current_stamp = robot_influx_bridge::line_protocol::stamp_to_nanoseconds(transform.header.stamp);
      if (has_last_stamp_ && current_stamp <= last_stamp_ns_) {
        return {};
      }

      last_stamp_ns_ = current_stamp;
      has_last_stamp_ = true;

      std::ostringstream line;
      line << build_series_name_with_frames(context);
      line << ' '
           << "trans_x=" << transform.transform.translation.x << ','
           << "trans_y=" << transform.transform.translation.y << ','
           << "trans_z=" << transform.transform.translation.z << ','
           << "rot_x=" << transform.transform.rotation.x << ','
           << "rot_y=" << transform.transform.rotation.y << ','
           << "rot_z=" << transform.transform.rotation.z << ','
           << "rot_w=" << transform.transform.rotation.w;

      robot_influx_bridge::append_timestamp_if_valid(line, transform.header.stamp);
      return {line.str()};
    } catch (const tf2::TransformException & ex) {
      if (clock_) {
        RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 2000,
        "Failed to lookup transform %s -> %s: %s",
        source_frame_.c_str(), target_frame_.c_str(), ex.what());
      }
    }

    return {};
  }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage &,
    const robot_influx_bridge::TranslatorContext &)
    override
  {
    // This translator is timer-driven, so it never processes serialized messages.
    return {};
  }

private:
  static double parse_double(
    const std::unordered_map<std::string, std::string> & config,
    const std::string & key,
    double default_value)
  {
    const auto it = config.find(key);
    if (it == config.end()) {
      return default_value;
    }
    try {
      return std::stod(it->second);
    } catch (const std::exception &) {
      return default_value;
    }
  }

  static int64_t parse_period(const std::unordered_map<std::string, std::string> & config)
  {
    const auto frequency_it = config.find("frequency_hz");
    if (frequency_it != config.end()) {
      try {
        const double freq = std::stod(frequency_it->second);
        if (freq > 0.0) {
          const double period_ns = 1'000'000'000.0 / freq;
          if (period_ns >= 1.0) {
            return static_cast<int64_t>(period_ns);
          }
          return 1;
        }
      } catch (const std::exception &) {
      }
    }

    const auto period_it = config.find("period_ns");
    if (period_it != config.end()) {
      try {
        const double period_ns = std::stod(period_it->second);
        if (period_ns >= 1.0) {
          return static_cast<int64_t>(period_ns);
        }
      } catch (const std::exception &) {
      }
    }

    return static_cast<int64_t>(1'000'000'000.0 / 50.0);
  }

  std::string build_series_name_with_frames(const robot_influx_bridge::TranslatorContext & context) const
  {
    const std::string measurement =
      robot_influx_bridge::line_protocol::resolve_measurement_name(context.measurement, "tf");

    std::vector<std::pair<std::string, std::string>> tags{
      context.static_tags.begin(), context.static_tags.end()};
    if (!source_frame_.empty()) {
      tags.emplace_back("source_frame", source_frame_);
    }
    if (!target_frame_.empty()) {
      tags.emplace_back("target_frame", target_frame_);
    }
    if (!parent_frame_.empty()) {
      tags.emplace_back("parent_frame", parent_frame_);
    }
    std::sort(tags.begin(), tags.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.first < rhs.first;
    });

    std::ostringstream series;
    series << robot_influx_bridge::line_protocol::escape_measurement(measurement);
    for (const auto & [key, value] : tags) {
      series << ',' << robot_influx_bridge::line_protocol::escape_field_key(key) << '='
             << robot_influx_bridge::line_protocol::escape_tag_value(value);
    }
    return series.str();
  }

  rclcpp::Logger logger_{rclcpp::get_logger("TfBasicTranslator")};
  std::string source_frame_;
  std::string target_frame_;
  std::string parent_frame_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  tf2::Duration lookup_timeout_{tf2::durationFromSec(0.1)};
  std::chrono::nanoseconds update_period_{std::chrono::milliseconds(20)};
  int64_t last_stamp_ns_{0};
  bool has_last_stamp_{false};
  bool configured_{false};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::TfBasicTranslator, robot_influx_bridge::TranslatorBase)
