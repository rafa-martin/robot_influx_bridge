#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>

#include <robot_influx_bridge/translator_base.hpp>


namespace robot_influx_bridge {

static inline std::string esc_tag(const std::string &s){
  std::string o; o.reserve(s.size());
  for(char c: s){ if(c==' '||c==','||c=='=') o.push_back('\\'); o.push_back(c); }
  return o;
}
static inline std::string esc_measure(const std::string &s){
  std::string o; o.reserve(s.size());
  for(char c: s){ if(c==' '||c==',' ) o.push_back('\\'); o.push_back(c); }
  return o;
}

class ImuTranslator final : public TranslatorBase {
public:
  std::string type_name() const override { return "sensor_msgs/msg/Imu"; }

  std::vector<std::string> to_line_protocol(
      const rclcpp::SerializedMessage & smsg,
      const Context & ctx) override {
    rclcpp::Serialization<sensor_msgs::msg::Imu> ser;
    sensor_msgs::msg::Imu msg;
    ser.deserialize_message(&smsg, &msg);

    const auto & m = ctx.measurement.empty() ? std::string("imu") : ctx.measurement;

    std::ostringstream line;
    line << esc_measure(m);

    // tags
    for (const auto & kv : ctx.static_tags) {
      line << ',' << esc_tag(kv.first) << '=' << esc_tag(kv.second);
    }
    // fields
    line << ' '
         << "ori_x=" << msg.orientation.x << ','
         << "ori_y=" << msg.orientation.y << ','
         << "ori_z=" << msg.orientation.z << ','
         << "ori_w=" << msg.orientation.w << ','
         << "ang_x=" << msg.angular_velocity.x << ','
         << "ang_y=" << msg.angular_velocity.y << ','
         << "ang_z=" << msg.angular_velocity.z << ','
         << "lin_x=" << msg.linear_acceleration.x << ','
         << "lin_y=" << msg.linear_acceleration.y << ','
         << "lin_z=" << msg.linear_acceleration.z;

    // timestamp in ns (prefer message header if present)
    int64_t t_ns = 0;
    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0) {
      t_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL
           + static_cast<int64_t>(msg.header.stamp.nanosec);
    }
    if (t_ns != 0) line << ' ' << t_ns;

    return {line.str()};
  }
};

} // namespace robot_influx_bridge

PLUGINLIB_EXPORT_CLASS(robot_influx_bridge::ImuTranslator, robot_influx_bridge::TranslatorBase)
