// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>
#include <robot_influx_bridge/line_protocol_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces
{

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
    line << robot_influx_bridge::build_series_name(context, "battery");
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
           << robot_influx_bridge::line_protocol::escape_field_key("location") << '='
           << robot_influx_bridge::line_protocol::escape_string_field_value(battery_message.location);
    }

    if (!battery_message.serial_number.empty()) {
      line << ','
           << robot_influx_bridge::line_protocol::escape_field_key("serial_number") << '='
           << robot_influx_bridge::line_protocol::escape_string_field_value(battery_message.serial_number);
    }

    robot_influx_bridge::append_timestamp_if_valid(line, battery_message.header.stamp);
    return {line.str()};
  }
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::BatteryStateTranslator, robot_influx_bridge::TranslatorBase)
