# Robot Influx Bridge Workspace

This workspace contains the ROS 2 packages that forward robot telemetry to InfluxDB.
It is designed around pluginlib-based translators so that any ROS 2 message type can
be converted into InfluxDB line protocol without modifying the core bridge.

## Repository Layout

| Directory | Description |
|-----------|-------------|
| `robot_influx_bridge` | Core bridge component that manages subscriptions and bulk uploads to InfluxDB. |
| `robot_influx_common_interfaces` | Collection of reusable translators for common ROS 2 messages. |
| `compose.yaml` | Optional docker-compose file for local testing with InfluxDB. |

## Getting Started

### Prerequisites

* ROS 2 Humble or later with a working colcon workspace.
* `libcurl` and `zlib` development headers (required by the writer implementation).
* Access credentials for an InfluxDB 2.x instance.

### Building the Workspace

Clone the repository into your ROS 2 workspace and build it with `colcon`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<your-org>/robot_influx_bridge.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --packages-up-to robot_influx_bridge robot_influx_common_interfaces
```

Source the workspace after building:

```bash
source install/setup.bash
```

### Launching the Bridge

The bridge is provided as a composable node. A sample launch file is included and can
be started with:

```bash
ros2 launch robot_influx_bridge robot_influx_bridge.launch.xml
```

By default the launch file loads the parameters from `config/robot_influx_bridge.yaml`.
Adapt the file to point at your InfluxDB instance and to configure the telemetry
mappings you want to publish.

## Configuration

Mappings are driven by ROS 2 parameters. A condensed example is shown below; the full
schema is generated from `bridge_parameters.yaml`.

```yaml
/**:
  ros__parameters:
    influx:
      url: "http://localhost:8086"
      org: "org"
      bucket: "bucket"
      token: "token"

    writer:
      max_batch: 1000
      max_queue: 100000
      flush_ms: 500
      use_gzip: false

    mappings_ids:
      - imu

    imu:
      topic: "/imu"
      type: "sensor_msgs/msg/Imu"
      measurement: "imu"
      plugin: "robot_influx_common_interfaces/ImuTranslator"
      max_rate: 200.0  # Limit processing to 200 Hz
      qos_history: keep_last
      qos_depth: 10
      qos_reliability: reliable
      qos_durability: volatile
      tags:
        - "frame_id=base_link"
```

* When the `plugin` field is empty the bridge resolves a default translator based on
  the message type. Provide the fully qualified plugin name to override the choice.
* Static tags can be defined per mapping. They are automatically added to every line
  produced by the translator.
* Enable `writer.use_gzip` to compress HTTP payloads before sending them to InfluxDB.
  This is recommended when the bridge publishes large batches.
* Each mapping can optionally throttle the processing rate via `max_rate` and override
  the ROS 2 subscription QoS (`qos_history`, `qos_depth`, `qos_reliability`,
  `qos_durability`).

## Creating Custom Translators

1. Create a new package (or use an existing one) that depends on `robot_influx_bridge`.
2. Implement a class deriving from `robot_influx_bridge::TranslatorBase`.
3. Register the class with `PLUGINLIB_EXPORT_CLASS` and update your package's
   `plugin_description.xml`.
4. Optionally extend `InfluxBridgeNode::resolvePluginClass` to register a default
   translator for the message type.

Common utilities for constructing line protocol strings live in
`robot_influx_common_interfaces::line_protocol`.

## Development Tips

* `InfluxWriter` maintains a background thread that batches measurements. The
  `last_error()` accessor surfaces the last network or HTTP error reported by the
  uploader.
* The bridge logs the list of available plugins on startup. If a mapping fails to
  resolve a translator, check that the package exporting the plugin is built and that
  its XML file is installed.
* For local testing, the included `compose.yaml` can spin up InfluxDB together with a
  Telegraf instance for inspecting the published data.

## License

The project is distributed under the MIT License. See `LICENSE` for details.
