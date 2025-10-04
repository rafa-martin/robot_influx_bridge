// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_influx_common_interfaces/line_protocol_utils.hpp>

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robot_influx_common_interfaces
{
namespace
{
std::string buildSeriesName(
  const robot_influx_bridge::TranslatorContext & context,
  const std::string & default_measurement)
{
  const std::string measurement =
    line_protocol::resolveMeasurementName(context.measurement, default_measurement);

  std::ostringstream series;
  series << line_protocol::escapeMeasurement(measurement);

  std::vector<std::pair<std::string, std::string>> tags{
    context.static_tags.begin(), context.static_tags.end()};
  std::sort(tags.begin(), tags.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.first < rhs.first;
  });

  for (const auto & [key, value] : tags) {
    series << ',' << line_protocol::escapeFieldKey(key) << '='
           << line_protocol::escapeTagValue(value);
  }

  return series.str();
}

void appendTimestampIfValid(std::ostringstream & stream, const builtin_interfaces::msg::Time & stamp)
{
  const int64_t time_ns = line_protocol::stampToNanoseconds(stamp);
  if (time_ns != 0) {
    stream << ' ' << time_ns;
  }
}

}  // namespace

class ImuTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "sensor_msgs/msg/Imu"; }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage & message,
    const robot_influx_bridge::TranslatorContext & context) override
  {
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    sensor_msgs::msg::Imu imu_message;
    serializer.deserialize_message(&message, &imu_message);

    std::ostringstream line;
    line << buildSeriesName(context, "imu");
    line << ' '
         << "ori_x=" << imu_message.orientation.x << ','
         << "ori_y=" << imu_message.orientation.y << ','
         << "ori_z=" << imu_message.orientation.z << ','
         << "ori_w=" << imu_message.orientation.w << ','
         << "ang_x=" << imu_message.angular_velocity.x << ','
         << "ang_y=" << imu_message.angular_velocity.y << ','
         << "ang_z=" << imu_message.angular_velocity.z << ','
         << "lin_x=" << imu_message.linear_acceleration.x << ','
         << "lin_y=" << imu_message.linear_acceleration.y << ','
         << "lin_z=" << imu_message.linear_acceleration.z;

    appendTimestampIfValid(line, imu_message.header.stamp);
    return {line.str()};
  }
};

class OdometryTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "nav_msgs/msg/Odometry"; }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage & message,
    const robot_influx_bridge::TranslatorContext & context) override
  {
    rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
    nav_msgs::msg::Odometry odometry_message;
    serializer.deserialize_message(&message, &odometry_message);

    std::ostringstream line;
    line << buildSeriesName(context, "odometry");
    line << ' '
         << "pos_x=" << odometry_message.pose.pose.position.x << ','
         << "pos_y=" << odometry_message.pose.pose.position.y << ','
         << "pos_z=" << odometry_message.pose.pose.position.z << ','
         << "ori_x=" << odometry_message.pose.pose.orientation.x << ','
         << "ori_y=" << odometry_message.pose.pose.orientation.y << ','
         << "ori_z=" << odometry_message.pose.pose.orientation.z << ','
         << "ori_w=" << odometry_message.pose.pose.orientation.w << ','
         << "lin_x=" << odometry_message.twist.twist.linear.x << ','
         << "lin_y=" << odometry_message.twist.twist.linear.y << ','
         << "lin_z=" << odometry_message.twist.twist.linear.z << ','
         << "ang_x=" << odometry_message.twist.twist.angular.x << ','
         << "ang_y=" << odometry_message.twist.twist.angular.y << ','
         << "ang_z=" << odometry_message.twist.twist.angular.z;

    appendTimestampIfValid(line, odometry_message.header.stamp);
    return {line.str()};
  }
};

class BatteryStateTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "sensor_msgs/msg/BatteryState"; }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage & message,
    const robot_influx_bridge::TranslatorContext & context) override
  {
    rclcpp::Serialization<sensor_msgs::msg::BatteryState> serializer;
    sensor_msgs::msg::BatteryState battery_message;
    serializer.deserialize_message(&message, &battery_message);

    std::ostringstream line;
    line << buildSeriesName(context, "battery");
    line << ' '
         << "voltage=" << battery_message.voltage << ','
         << "current=" << battery_message.current << ','
         << "charge=" << battery_message.charge << ','
         << "capacity=" << battery_message.capacity << ','
         << "design_capacity=" << battery_message.design_capacity << ','
         << "percentage=" << battery_message.percentage << ','
         << "temperature=" << battery_message.temperature << ','
         << "status=" << static_cast<int>(battery_message.power_supply_status) << 'i' << ','
         << "health=" << static_cast<int>(battery_message.power_supply_health) << 'i' << ','
         << "technology=" << static_cast<int>(battery_message.power_supply_technology) << 'i' << ','
         << "present=" << (battery_message.present ? "true" : "false");

    if (!battery_message.location.empty()) {
      line << ','
           << line_protocol::escapeFieldKey("location") << '='
           << line_protocol::escapeStringFieldValue(battery_message.location);
    }

    if (!battery_message.serial_number.empty()) {
      line << ','
           << line_protocol::escapeFieldKey("serial_number") << '='
           << line_protocol::escapeStringFieldValue(battery_message.serial_number);
    }

    appendTimestampIfValid(line, battery_message.header.stamp);
    return {line.str()};
  }
};

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

    const double buffer_time = parseDouble(context.custom_config, "buffer_time_sec", 10.0);
    lookup_timeout_ = tf2::durationFromSec(parseDouble(context.custom_config, "lookup_timeout_sec", 0.1));
    update_period_ = std::chrono::nanoseconds(parsePeriod(context.custom_config));

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

      const int64_t current_stamp = line_protocol::stampToNanoseconds(transform.header.stamp);
      if (has_last_stamp_ && current_stamp <= last_stamp_ns_) {
        return {};
      }

      last_stamp_ns_ = current_stamp;
      has_last_stamp_ = true;

      std::ostringstream line;
      line << buildSeriesNameWithFrames(context);
      line << ' '
           << "trans_x=" << transform.transform.translation.x << ','
           << "trans_y=" << transform.transform.translation.y << ','
           << "trans_z=" << transform.transform.translation.z << ','
           << "rot_x=" << transform.transform.rotation.x << ','
           << "rot_y=" << transform.transform.rotation.y << ','
           << "rot_z=" << transform.transform.rotation.z << ','
           << "rot_w=" << transform.transform.rotation.w;

      appendTimestampIfValid(line, transform.header.stamp);
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
  static double parseDouble(
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

  static int64_t parsePeriod(const std::unordered_map<std::string, std::string> & config)
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

  std::string buildSeriesNameWithFrames(const robot_influx_bridge::TranslatorContext & context) const
  {
    const std::string measurement =
      line_protocol::resolveMeasurementName(context.measurement, "tf");

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
    series << line_protocol::escapeMeasurement(measurement);
    for (const auto & [key, value] : tags) {
      series << ',' << line_protocol::escapeFieldKey(key) << '='
             << line_protocol::escapeTagValue(value);
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

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::ImuTranslator, robot_influx_bridge::TranslatorBase)
PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::OdometryTranslator, robot_influx_bridge::TranslatorBase)
PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::BatteryStateTranslator, robot_influx_bridge::TranslatorBase)
PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::TfBasicTranslator, robot_influx_bridge::TranslatorBase)
