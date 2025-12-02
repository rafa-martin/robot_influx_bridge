// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces
{

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
    line << robot_influx_bridge::build_series_name(context, "odometry");
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

    robot_influx_bridge::append_timestamp_if_valid(line, odometry_message.header.stamp);
    return {line.str()};
  }
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::OdometryTranslator, robot_influx_bridge::TranslatorBase)
