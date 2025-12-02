// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces
{

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
    line << robot_influx_bridge::build_series_name(context, "imu");
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

    robot_influx_bridge::append_timestamp_if_valid(line, imu_message.header.stamp);
    return {line.str()};
  }
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::ImuTranslator, robot_influx_bridge::TranslatorBase)
