// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <robot_influx_common_interfaces/line_protocol_utils.hpp>

#include <algorithm>
#include <sstream>
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

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::ImuTranslator, robot_influx_bridge::TranslatorBase)
PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::OdometryTranslator, robot_influx_bridge::TranslatorBase)
PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::BatteryStateTranslator, robot_influx_bridge::TranslatorBase)
