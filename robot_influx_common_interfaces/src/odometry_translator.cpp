// Copyright 2025, Rafael Martin. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of Rafael Martin nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <robot_influx_bridge/translator_base.hpp>
#include <robot_influx_bridge/translator_utils.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_common_interfaces {

class OdometryTranslator final : public robot_influx_bridge::TranslatorBase {
public:
    std::string type_name() const override { return "nav_msgs/msg/Odometry"; }

    std::vector<std::string> to_line_protocol(const rclcpp::SerializedMessage& message,
                                              const robot_influx_bridge::TranslatorContext& context) override {
        rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
        nav_msgs::msg::Odometry odometry_message;
        serializer.deserialize_message(&message, &odometry_message);

        std::ostringstream line;
        line << robot_influx_bridge::build_series_name(context, "odometry");
        line << ' ' << "pos_x=" << odometry_message.pose.pose.position.x << ','
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

} // namespace robot_influx_common_interfaces

PLUGINLIB_EXPORT_CLASS(robot_influx_common_interfaces::OdometryTranslator, robot_influx_bridge::TranslatorBase)
