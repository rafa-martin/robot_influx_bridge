#include <robot_influx_bridge/influx_bridge.hpp>

namespace robot_influx_bridge {


InfluxBridgeNode::InfluxBridgeNode(rclcpp::NodeOptions const& options)
: Node("influx_bridge", options)
{
  param_listener_ = std::make_shared<influx_bridge::ParamListener>(get_node_parameters_interface());
  param_listener_->setUserCallback(
    [this](influx_bridge::Params const& params) {
      (void)params;
      RCLCPP_INFO(this->get_logger(), "Parameters updated");
    });

  // take snapshot of the current parameters
  params_ = param_listener_->get_params();

  // writer
  writer_ = std::make_unique<robot_influx_bridge::InfluxWriter>(
    params_.influx.url,
    params_.influx.org,
    params_.influx.bucket,
    params_.influx.token,
    static_cast<size_t>(params_.writer.max_batch),
    static_cast<size_t>(params_.writer.max_queue),
    std::chrono::milliseconds(params_.writer.flush_ms));

  // translator loader
  loader_ = std::make_unique<pluginlib::ClassLoader<TranslatorBase>>(
    "robot_influx_bridge",
    "robot_influx_bridge::TranslatorBase");

  // mappings
  RCLCPP_INFO(this->get_logger(), "Loaded plugins:");
  for (auto const& [id, mapping] : params_.mappings_ids_map) add_mapping_from_params(id);

}

void InfluxBridgeNode::add_mapping_from_params(const std::string& id)
{
  const auto & entry = params_.mappings_ids_map.at(id);

  Mapping m;
  m.topic = entry.topic;
  m.type  = entry.type;
  m.ctx.measurement = entry.measurement;

  RCLCPP_INFO(this->get_logger(), "Adding mapping %s: %s [%s] -> %s",
    id.c_str(), m.topic.c_str(), m.type.c_str(), m.ctx.measurement.c_str());

  // collect tags from nested mapped params: tag_keys -> value
  // for (const auto& k : entry.tag_keys) {
  //   const auto& tag_entry = entry.tags.tag_keys_map.at(k);
  //   m.ctx.static_tags[k] = tag_entry.value;
  // }

  // plugin class by param
  std::string plugin_class = m.type;
  try {
    if (!loader_->isClassAvailable(plugin_class)) {
      RCLCPP_ERROR(this->get_logger(), "Plugin class %s not found", plugin_class.c_str());
      RCLCPP_ERROR(this->get_logger(), "Available plugins:");
      for (const auto& name : loader_->getDeclaredClasses()) {
        RCLCPP_ERROR(this->get_logger(), "  %s", name.c_str());
      }
      return;
    }

  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Error checking plugin %s: %s", plugin_class.c_str(), ex.what());
    return;
  }
  m.translator = loader_->createSharedInstance(plugin_class);

  // subscribe generically
  rclcpp::QoS qos( rclcpp::KeepLast(10) );
  auto cb = [this, ctx=m.ctx, tr=m.translator](const std::shared_ptr<rclcpp::SerializedMessage> msg){
    RCLCPP_INFO(get_logger(), "Received message of type %s", tr->type_name().c_str());
    for(const auto & line : tr->to_line_protocol(*msg, ctx)){
      if(!writer_->enqueue(line)){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, "Queue full. Dropping line.");
      }
    }
  };
  m.sub = create_generic_subscription(m.topic, m.translator->type_name(), qos, cb);

  mappings_.push_back(std::move(m));
  RCLCPP_INFO(this->get_logger(), "Subscribed %s [%s]", mappings_.back().topic.c_str(), mappings_.back().type.c_str());
}

}  // namespace robot_influx_bridge


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robot_influx_bridge::InfluxBridgeNode)
