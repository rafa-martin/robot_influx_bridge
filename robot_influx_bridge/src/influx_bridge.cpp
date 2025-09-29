#include <robot_influx_bridge/influx_bridge.hpp>

#include <unordered_map>

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
  translator_loader_ = std::make_unique<pluginlib::ClassLoader<TranslatorBase>>(
    "robot_influx_bridge",
    "robot_influx_bridge::TranslatorBase");

  // mappings
  RCLCPP_INFO(this->get_logger(), "Loaded plugins:");
  for (const auto & mapping_entry : params_.mappings_ids_map) {
    addMappingFromParams(mapping_entry.first);
  }

}

void InfluxBridgeNode::addMappingFromParams(const std::string& mapping_id)
{
  const auto & entry = params_.mappings_ids_map.at(mapping_id);

  TopicMapping mapping;
  mapping.topic_name = entry.topic;
  mapping.translator_context.measurement = entry.measurement;
  std::string plugin_class = entry.translator;

  RCLCPP_INFO(this->get_logger(), "Adding mapping %s: %s [%s] -> %s",
    mapping_id.c_str(), mapping.topic_name.c_str(), plugin_class.c_str(), mapping.translator_context.measurement.c_str());

  // collect tags from nested mapped params: tag_keys -> value
  for (const auto& k : entry.tag_keys) {
    // split at first '='
    const auto pos = k.find('=');
    if (pos == std::string::npos || pos == 0 || pos == k.size() - 1) {
      RCLCPP_ERROR(this->get_logger(), "Invalid tag key '%s' in mapping %s. Expected format 'key=value'. Skipping.", k.c_str(), mapping_id.c_str());
      continue;
    }
    const std::string key = k.substr(0, pos);
    const std::string value = k.substr(pos + 1);
    mapping.translator_context.static_tags[key] = value;
  }

  // plugin class by param
  if (plugin_class.empty()) {
    RCLCPP_ERROR(this->get_logger(),
      "Mapping %s did not resolve to a plugin. Provide 'plugin' parameter or use a supported message type.",
      mapping_id.c_str());
    return;
  }
  try {
    if (!translator_loader_->isClassAvailable(plugin_class)) {
      RCLCPP_ERROR(this->get_logger(), "Plugin class %s not found", plugin_class.c_str());
      RCLCPP_ERROR(this->get_logger(), "Available plugins:");
      for (const auto& name : translator_loader_->getDeclaredClasses()) {
        RCLCPP_ERROR(this->get_logger(), "  %s", name.c_str());
      }
      return;
    }

  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Error checking plugin %s: %s", plugin_class.c_str(), ex.what());
    return;
  }
  mapping.translator = translator_loader_->createSharedInstance(plugin_class);

  // subscribe generically
  rclcpp::QoS qos( rclcpp::KeepLast(10) );
  auto callback = [this, context=mapping.translator_context, translator=mapping.translator](const std::shared_ptr<rclcpp::SerializedMessage> message){
    for(const auto & line : translator->to_line_protocol(*message, context)){
      if(!writer_->enqueue(line)){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, "Queue full. Dropping line.");
      }
    }
  };
  mapping.subscription = create_generic_subscription(mapping.topic_name, mapping.translator->type_name(), qos, callback);

  topic_mappings_.push_back(std::move(mapping));
  RCLCPP_INFO(this->get_logger(), "Subscribed %s [%s]",
    topic_mappings_.back().topic_name.c_str(),
    topic_mappings_.back().translator->type_name().c_str());
}

}  // namespace robot_influx_bridge


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robot_influx_bridge::InfluxBridgeNode)
