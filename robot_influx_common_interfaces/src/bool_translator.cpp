// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces
{

class BoolTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "std_msgs/msg/Bool"; }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage & message,
    const robot_influx_bridge::TranslatorContext & context) override
  {
    rclcpp::Serialization<std_msgs::msg::Bool> serializer;
    std_msgs::msg::Bool bool_message;
    serializer.deserialize_message(&message, &bool_message);

    std::ostringstream line;
    line << robot_influx_bridge::build_series_name(context, "bool");
    line << ' '
         << "value=" << (bool_message.data ? "true" : "false");

    // std_msgs/Bool doesn't have a header/timestamp, so no timestamp is appended
    return {line.str()};
  }
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::BoolTranslator, robot_influx_bridge::TranslatorBase)
