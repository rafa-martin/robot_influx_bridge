// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces
{

class TwistTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "geometry_msgs/msg/Twist"; }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage & message,
    const robot_influx_bridge::TranslatorContext & context) override
  {
    rclcpp::Serialization<geometry_msgs::msg::Twist> serializer;
    geometry_msgs::msg::Twist twist_message;
    serializer.deserialize_message(&message, &twist_message);

    std::ostringstream line;
    line << robot_influx_bridge::build_series_name(context, "cmd_vel");
    line << ' '
         << "linear_x=" << twist_message.linear.x << ','
         << "linear_y=" << twist_message.linear.y << ','
         << "linear_z=" << twist_message.linear.z << ','
         << "angular_x=" << twist_message.angular.x << ','
         << "angular_y=" << twist_message.angular.y << ','
         << "angular_z=" << twist_message.angular.z;

    // Twist messages don't have a header/timestamp, so no timestamp is appended
    return {line.str()};
  }
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::TwistTranslator, robot_influx_bridge::TranslatorBase)
