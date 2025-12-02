// SPDX-License-Identifier: MIT

#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces
{

class NavSatFixTranslator final : public robot_influx_bridge::TranslatorBase
{
public:
  std::string type_name() const override { return "sensor_msgs/msg/NavSatFix"; }

  std::vector<std::string> to_line_protocol(
    const rclcpp::SerializedMessage & message,
    const robot_influx_bridge::TranslatorContext & context) override
  {
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
    sensor_msgs::msg::NavSatFix gps_message;
    serializer.deserialize_message(&message, &gps_message);

    std::ostringstream line;
    line << robot_influx_bridge::build_series_name(context, "gps");
    line << ' '
         << "latitude=" << gps_message.latitude << ','
         << "longitude=" << gps_message.longitude << ','
         << "altitude=" << gps_message.altitude << ','
         << "status=" << static_cast<int>(gps_message.status.status) << 'i' << ','
         << "service=" << static_cast<int>(gps_message.status.service) << 'i';

    // Add position covariance if available
    if (!gps_message.position_covariance.empty()) {
      line << ','
           << "cov_xx=" << gps_message.position_covariance[0] << ','
           << "cov_yy=" << gps_message.position_covariance[4] << ','
           << "cov_zz=" << gps_message.position_covariance[8] << ','
           << "cov_type=" << static_cast<int>(gps_message.position_covariance_type) << 'i';
    }

    robot_influx_bridge::append_timestamp_if_valid(line, gps_message.header.stamp);
    return {line.str()};
  }
};

}  // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::NavSatFixTranslator, robot_influx_bridge::TranslatorBase)
