// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces
{

class JointStateTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "sensor_msgs/msg/JointState"; }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage & message,
    const robot_influx_bridge::TranslatorContext & context) override
  {
    rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
    sensor_msgs::msg::JointState joint_message;
    serializer.deserialize_message(&message, &joint_message);

    std::vector<std::string> lines;
    
    // Create a separate line for each joint
    for (size_t i = 0; i < joint_message.name.size(); ++i) {
      std::ostringstream line;
      line << robot_influx_bridge::build_series_name(context, "joint_states");
      
      // Add joint name as a tag
      line << ",joint=" << joint_message.name[i];
      
      line << ' ';
      
      // Add position if available
      if (i < joint_message.position.size()) {
        line << "position=" << joint_message.position[i];
      }
      
      // Add velocity if available
      if (i < joint_message.velocity.size()) {
        if (i < joint_message.position.size()) line << ',';
        line << "velocity=" << joint_message.velocity[i];
      }
      
      // Add effort if available
      if (i < joint_message.effort.size()) {
        if (i < joint_message.position.size() || i < joint_message.velocity.size()) line << ',';
        line << "effort=" << joint_message.effort[i];
      }

      robot_influx_bridge::append_timestamp_if_valid(line, joint_message.header.stamp);
      lines.push_back(line.str());
    }

    return lines;
  }
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::JointStateTranslator, robot_influx_bridge::TranslatorBase)
