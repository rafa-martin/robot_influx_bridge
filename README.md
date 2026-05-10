# Robot Influx Bridge Workspace

This workspace contains the ROS 2 packages that forward robot telemetry to InfluxDB.
It is designed around pluginlib-based translators so that any ROS 2 message type can
be converted into InfluxDB line protocol without modifying the core bridge.

## Repository Layout

| Directory | Description |
|-----------|-------------|
| `robot_influx_bridge` | Core bridge component that manages subscriptions and bulk uploads to InfluxDB. |
| `robot_influx_common_interfaces` | Collection of reusable translators for common ROS 2 messages. |
| `container/` | Docker compose setup for local testing with InfluxDB and Telegraf. |

## Getting Started

### Prerequisites

* ROS 2 Humble or later with a working colcon workspace.
* `libcurl` and `zlib` development headers (required by the writer implementation).
* Access credentials for an InfluxDB 2.x instance (or use the provided Docker setup).
* Docker and Docker Compose (optional, for local testing setup).

### Building the Workspace

Clone the repository into your ROS 2 workspace and build it with `colcon`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rafa-martin/robot_influx_bridge.git
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

Mappings are driven by ROS 2 parameters. The actual configuration is shown below, with
the full schema defined in `bridge_parameters.yaml`.

```yaml
/**:
  ros__parameters:
    influx:
      url: "http://localhost:8087"
      org: "myorg"                    # Set by Docker setup
      bucket: "mybucket"              # Set by Docker setup
      token: "my-super-secret-auth-token"

    writer:
      max_batch: 1000
      max_queue: 100000
      flush_ms: 5000                  # 5 second flush interval
      use_gzip: true                  # Enabled for better performance
      persistence_path: "/tmp/robot_influx_bridge"
      persistence_max_megabytes: 128

    mappings_ids:
      - imu
      - tf_map

    imu:
      topic: "/robot/imu/data"
      translator: "robot_influx_common_interfaces::ImuTranslator"
      measurement: "imu"
      max_rate: 200.0                 # Limit processing to 200 Hz
      qos_history: keep_last
      qos_depth: 10
      qos_reliability: best_effort    # For high-frequency IMU data
      qos_durability: volatile
      tag_keys:
        - "frame_id=imu_link"

    tf_map:
      topic: ""                       # TF translator doesn't use topic subscription
      translator: "robot_influx_common_interfaces::TfBasicTranslator"
      measurement: "tf"
      max_rate: 0.0                   # No rate limiting
      qos_history: keep_last
      qos_depth: 10
      qos_reliability: reliable
      qos_durability: volatile
      tag_keys:
        - "frame_id=map"
      custom_config:
        - "source_frame=robot_base_link"
        - "target_frame=robot_odom"
        - "parent_frame=robot_odom"
        - "buffer_time_sec=10.0"
        - "lookup_timeout_sec=0.1"
        - "frequency_hz=50.0"
```

* When the `translator` field is empty, the bridge resolves a default translator based on
  the message type. Provide the fully qualified translator class name to override.
* Static tags can be defined per mapping using `tag_keys`. They are automatically added
  to every line produced by the translator.
* Enable `writer.use_gzip` to compress HTTP payloads before sending them to InfluxDB.
  This is recommended when the bridge publishes large batches.
* Each mapping can optionally throttle the processing rate via `max_rate` and override
  the ROS 2 subscription QoS (`qos_history`, `qos_depth`, `qos_reliability`,
  `qos_durability`).
* The TF translator uses `custom_config` to specify transform lookup parameters instead
  of subscribing to a topic.
* The bridge includes persistence functionality to store batches locally when InfluxDB
  is unavailable. Configure `writer.persistence_path` and `writer.persistence_max_megabytes`
  to control this behavior.

## Available Translators

The `robot_influx_common_interfaces` package provides several built-in translators:

| Translator Class | Message Type | Description |
|------------------|--------------|-------------|
| `robot_influx_common_interfaces::ImuTranslator` | `sensor_msgs/msg/Imu` | Converts IMU data (orientation, angular velocity, linear acceleration) |
| `robot_influx_common_interfaces::OdometryTranslator` | `nav_msgs/msg/Odometry` | Converts robot odometry (pose, twist) |
| `robot_influx_common_interfaces::BatteryStateTranslator` | `sensor_msgs/msg/BatteryState` | Converts battery status and metrics |
| `robot_influx_common_interfaces::TfBasicTranslator` | TF transforms | Periodically looks up and logs transform relationships |
| `robot_influx_common_interfaces::TwistTranslator` | `geometry_msgs/msg/Twist` | Converts linear and angular velocity commands |
| `robot_influx_common_interfaces::BoolTranslator` | `std_msgs/msg/Bool` | Converts a boolean value to a single field |
| `robot_influx_common_interfaces::NavSatFixTranslator` | `sensor_msgs/msg/NavSatFix` | Converts GPS fix (latitude, longitude, altitude, covariance) |
| `robot_influx_common_interfaces::JointStateTranslator` | `sensor_msgs/msg/JointState` | Converts joint position, velocity, and effort (one line per joint) |

All translators support custom configuration via the `tag_keys` and `custom_config` parameters.

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
* The bridge publishes periodic diagnostics via `diagnostic_updater`, which
  emits `diagnostic_msgs/msg/DiagnosticArray` messages on `/diagnostics`. The
  status includes keys such as `queue_depth`, `queue_utilization`,
  `persisted_batches`, `persistence_bytes_used`, and `last_error`, making it easy
  to monitor the health of the uploader.
* The bridge logs the list of available plugins on startup. If a mapping fails to
  resolve a translator, check that the package exporting the plugin is built and that
  its XML file is installed.

## Local Testing with Docker

For local testing, the included `container/compose.yaml` can spin up InfluxDB together
with a Telegraf instance for inspecting the published data.

```bash
cd container/
docker compose up -d
```

This creates:
- InfluxDB 2.x on port 8086 with admin credentials (`admin`/`adminpassword`)
- Organization: `myorg`, Bucket: `mybucket`
- Auth token: `my-super-secret-auth-token`
- Telegraf proxy on port 8087 for data collection
- 1 week data retention policy

The bridge configuration is set to connect to Telegraf on port 8087, which forwards
data to InfluxDB. You can access the InfluxDB UI at http://localhost:8086 to view
the collected telemetry data.

## License

The project is distributed under the BSD-3-Clause License. See `LICENSE` for details.
