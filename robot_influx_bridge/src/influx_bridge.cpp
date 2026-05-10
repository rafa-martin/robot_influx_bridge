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

#include <robot_influx_bridge/influx_bridge.hpp>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;

namespace robot_influx_bridge {

InfluxBridgeNode::InfluxBridgeNode(rclcpp::NodeOptions const& options)
    : Node("influx_bridge", options)
    , diagnostics_updater_(this) {
    param_listener_ = std::make_shared<influx_bridge::ParamListener>(get_node_parameters_interface());
    param_listener_->setUserCallback([this](influx_bridge::Params const& params) {
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
        params_.writer.use_gzip,
        params_.writer.persistence_path,
        static_cast<size_t>(params_.writer.persistence_max_megabytes));

    diagnostics_updater_.setHardwareID(this->get_fully_qualified_name());
    diagnostics_updater_.add("Influx Writer", this, &InfluxBridgeNode::publish_diagnostics);
    diagnostics_timer_ = this->create_wall_timer(1s, [this]() { diagnostics_updater_.force_update(); });
    diagnostics_updater_.force_update();

    // translator loader
    translator_loader_ = std::make_unique<pluginlib::ClassLoader<TranslatorBase>>(
        "robot_influx_bridge", "robot_influx_bridge::TranslatorBase");

    // mappings — iterate the ordered list to respect declaration order
    for (const auto& mapping_id : params_.mappings_ids) {
        add_mapping_from_params(mapping_id);
    }
    RCLCPP_INFO(this->get_logger(), "All mappings loaded.");
}

void InfluxBridgeNode::add_mapping_from_params(const std::string& mapping_id) {
    const auto& entry = params_.mappings_ids_map.at(mapping_id);

    TopicMapping mapping;
    mapping.topic_name = entry.topic;
    mapping.translator_context.measurement = entry.measurement;
    std::string plugin_class = entry.translator;

    RCLCPP_INFO(this->get_logger(),
                "Adding mapping %s: %s [%s] -> %s",
                mapping_id.c_str(),
                mapping.topic_name.c_str(),
                plugin_class.c_str(),
                mapping.translator_context.measurement.c_str());

    // collect tags from nested mapped params: tag_keys -> value
    for (const auto& k : entry.tag_keys) {
        // split at first '='
        const auto pos = k.find('=');
        if (pos == std::string::npos || pos == 0 || pos == k.size() - 1) {
            RCLCPP_ERROR(this->get_logger(),
                         "Invalid tag key '%s' in mapping %s. Expected format 'key=value'. Skipping.",
                         k.c_str(),
                         mapping_id.c_str());
            continue;
        }
        const std::string key = k.substr(0, pos);
        const std::string value = k.substr(pos + 1);
        mapping.translator_context.static_tags[key] = value;
    }

    for (const auto& item : entry.custom_config) {
        const auto pos = item.find('=');
        if (pos == std::string::npos || pos == 0 || pos == item.size() - 1) {
            RCLCPP_ERROR(this->get_logger(),
                         "Invalid custom config entry '%s' in mapping %s. Expected format 'key=value'. Skipping.",
                         item.c_str(),
                         mapping_id.c_str());
            continue;
        }
        const std::string key = item.substr(0, pos);
        const std::string value = item.substr(pos + 1);
        mapping.translator_context.custom_config[key] = value;
    }

    // plugin class by param
    if (plugin_class.empty()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Mapping %s did not resolve to a plugin. Provide 'translator' parameter or use a supported message type.",
            mapping_id.c_str());
        return;
    }
    try {
        if (!translator_loader_->isClassAvailable(plugin_class)) {
            RCLCPP_ERROR(this->get_logger(), "Plugin class %s not found", plugin_class.c_str());
            RCLCPP_INFO(this->get_logger(), "Available plugins:");
            for (const auto& name : translator_loader_->getDeclaredClasses()) {
                RCLCPP_INFO(this->get_logger(), "  %s", name.c_str());
            }
            return;
        }
    }
    catch (const pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Error checking plugin %s: %s", plugin_class.c_str(), ex.what());
        return;
    }
    mapping.translator = translator_loader_->createSharedInstance(plugin_class);

    mapping.translator->configure(*this, mapping.translator_context);

    if (mapping.translator->requires_subscription() && mapping.topic_name.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Mapping %s requires a topic, but none was provided. Set the 'topic' parameter.",
                     mapping_id.c_str());
        return;
    }

    if (mapping.translator->requires_subscription() && entry.max_rate > 0.0) {
        mapping.downsample_state = std::make_shared<TopicMapping::DownsampleState>();
        const double min_interval_seconds = 1.0 / entry.max_rate;
        int64_t min_interval_ns = static_cast<int64_t>(std::llround(min_interval_seconds * 1'000'000'000.0));
        if (min_interval_ns <= 0) {
            min_interval_ns = 1;
        }
        mapping.downsample_state->min_gap = rclcpp::Duration::from_nanoseconds(min_interval_ns);
    }

    if (auto timer_period = mapping.translator->get_timer_period(mapping.translator_context)) {
        if (timer_period->count() <= 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Mapping %s requested non-positive timer period. Ignoring timer.",
                        mapping_id.c_str());
        }
        else {
            mapping.timer = this->create_wall_timer(
                *timer_period, [this, translator = mapping.translator, context = mapping.translator_context]() {
                    const auto now = this->get_clock()->now();
                    for (const auto& line : translator->on_timer(context, now)) {
                        if (!writer_->enqueue(line)) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, "Queue full. Dropping line.");
                        }
                    }
                });
            RCLCPP_INFO(this->get_logger(),
                        "Mapping %s configured with periodic translator callback every %.3f ms.",
                        mapping_id.c_str(),
                        static_cast<double>(timer_period->count()) / 1'000'000.0);
        }
    }

    if (mapping.translator->requires_subscription()) {
        // subscribe generically
        size_t depth = static_cast<size_t>(entry.qos_depth);
        if (depth == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Mapping %s configured with QoS depth 0. Falling back to depth 1.",
                        mapping_id.c_str());
            depth = 1U;
        }
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
        if (entry.qos_history == "keep_all") {
            qos = rclcpp::QoS(rclcpp::KeepAll());
        }
        else if (entry.qos_history != "keep_last") {
            RCLCPP_WARN(this->get_logger(),
                        "Mapping %s configured with unsupported QoS history '%s'. Falling back to keep_last.",
                        mapping_id.c_str(),
                        entry.qos_history.c_str());
        }
        if (entry.qos_reliability == "best_effort") {
            qos.best_effort();
        }
        else if (entry.qos_reliability == "reliable") {
            qos.reliable();
        }
        else {
            RCLCPP_WARN(this->get_logger(),
                        "Mapping %s configured with unsupported QoS reliability '%s'. Falling back to reliable.",
                        mapping_id.c_str(),
                        entry.qos_reliability.c_str());
            qos.reliable();
        }
        if (entry.qos_durability == "transient_local") {
            qos.transient_local();
        }
        else if (entry.qos_durability == "volatile") {
            qos.durability_volatile();
        }
        else {
            RCLCPP_WARN(this->get_logger(),
                        "Mapping %s configured with unsupported QoS durability '%s'. Falling back to volatile.",
                        mapping_id.c_str(),
                        entry.qos_durability.c_str());
            qos.durability_volatile();
        }
        if (mapping.downsample_state && mapping.downsample_state->min_gap.nanoseconds() > 0) {
            const double effective_hz = entry.max_rate;
            const double min_interval_ms =
                static_cast<double>(mapping.downsample_state->min_gap.nanoseconds()) / 1'000'000.0;
            RCLCPP_INFO(this->get_logger(),
                        "Downsampling topic %s to at most %.3f Hz (minimum interval %.3f ms)",
                        mapping.topic_name.c_str(),
                        effective_hz,
                        min_interval_ms);
        }

        auto callback = [this,
                         context = mapping.translator_context,
                         translator = mapping.translator,
                         state = mapping.downsample_state](const std::shared_ptr<rclcpp::SerializedMessage> message) {
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
            for (const auto& line : translator->to_line_protocol(*message, context)) {
                if (!writer_->enqueue(line)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, "Queue full. Dropping line.");
                }
            }
        };
        mapping.subscription =
            create_generic_subscription(mapping.topic_name, mapping.translator->type_name(), qos, callback);

        RCLCPP_INFO(this->get_logger(),
                    "Subscribed %s [%s]",
                    mapping.topic_name.c_str(),
                    mapping.translator->type_name().c_str());
    }

    topic_mappings_.push_back(std::move(mapping));
}

void InfluxBridgeNode::publish_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");

    if (!writer_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Writer unavailable");
        return;
    }

    const size_t depth = writer_->queue_depth();
    const size_t max_queue = writer_->max_queue_size();
    const uintmax_t persisted_bytes = writer_->persistence_bytes_used();
    const size_t persisted_batches = writer_->pending_batch_count();
    const std::string last_error = writer_->last_error();
    const double utilization = max_queue > 0 ? static_cast<double>(depth) / static_cast<double>(max_queue) : 0.0;

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> issues;
    auto flag_issue = [&](uint8_t issue_level, const std::string& message) {
        if (issue_level > level) {
            level = issue_level;
        }
        if (!message.empty()) {
            issues.push_back(message);
        }
    };

    if (utilization >= 0.95) {
        flag_issue(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Queue nearly full");
    }
    else if (utilization >= 0.75) {
        flag_issue(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Queue utilization high");
    }

    if (persisted_batches > 0) {
        flag_issue(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Using disk persistence");
    }

    if (!last_error.empty()) {
        flag_issue(diagnostic_msgs::msg::DiagnosticStatus::WARN, std::string("Last error: ") + last_error);
    }

    std::string summary = "OK";
    if (!issues.empty()) {
        std::ostringstream oss;
        for (size_t i = 0; i < issues.size(); ++i) {
            if (i > 0) {
                oss << "; ";
            }
            oss << issues[i];
        }
        summary = oss.str();
    }

    status.summary(level, summary);

    std::ostringstream util_stream;
    util_stream << std::fixed << std::setprecision(2) << (utilization * 100.0);

    status.add("queue_depth", std::to_string(depth));
    status.add("max_queue", std::to_string(max_queue));
    status.add("queue_utilization", util_stream.str() + "%");
    status.add("persisted_batches", std::to_string(persisted_batches));
    status.add("persistence_bytes_used", std::to_string(persisted_bytes));
    status.add("last_error", last_error.empty() ? std::string("(none)") : last_error);
}

} // namespace robot_influx_bridge

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robot_influx_bridge::InfluxBridgeNode)
