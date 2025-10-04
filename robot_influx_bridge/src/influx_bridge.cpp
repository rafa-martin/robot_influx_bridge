#include <robot_influx_bridge/influx_bridge.hpp>

#include <cstdint>

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
    this->get_logger(),
    params_.influx.url,
    params_.influx.org,
    params_.influx.bucket,
    params_.influx.token,
    static_cast<size_t>(params_.writer.max_batch),
    static_cast<size_t>(params_.writer.max_queue),
    std::chrono::milliseconds(params_.writer.flush_ms),
    params_.writer.use_gzip);

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

  if (entry.min_interval_ms > 0) {
    mapping.downsample_state = std::make_shared<TopicMapping::DownsampleState>();
    const int64_t min_interval_ns = static_cast<int64_t>(entry.min_interval_ms) * 1000000LL;
    mapping.downsample_state->min_gap = rclcpp::Duration::from_nanoseconds(min_interval_ns);
  }

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
  if (mapping.downsample_state && mapping.downsample_state->min_gap.nanoseconds() > 0) {
    const double effective_hz = 1000.0 / static_cast<double>(entry.min_interval_ms);
    RCLCPP_WARN(this->get_logger(),
      "Downsampling topic %s to at most %.3f Hz (minimum interval %ld ms)",
      mapping.topic_name.c_str(),
      effective_hz,
      static_cast<long>(entry.min_interval_ms));
  }

  auto callback = [this,
      context=mapping.translator_context,
      translator=mapping.translator,
      state=mapping.downsample_state](const std::shared_ptr<rclcpp::SerializedMessage> message){
    if (state && state->min_gap.nanoseconds() > 0) {
      const auto now = this->get_clock()->now();
      if (state->has_last_emit) {
        const auto delta = now - state->last_emit;
        if (delta < state->min_gap) {
          return;
        }
      }
      state->last_emit = now;
      state->has_last_emit = true;
    }
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
