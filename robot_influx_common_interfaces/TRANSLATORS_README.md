# Translators Overview

This package provides a collection of reusable translators for common ROS 2 message types. Each translator converts ROS 2 messages into InfluxDB line protocol format for telemetry storage and analysis.

## Available Translators

| Translator | Message Type | InfluxDB Fields | Configuration Notes |
|------------|--------------|-----------------|-------------------|
| **ImuTranslator** | `sensor_msgs/msg/Imu` | ori_xyzw, ang_xyz, lin_xyz | Static tags via `tag_keys` |
| **OdometryTranslator** | `nav_msgs/msg/Odometry` | pos_xyz, ori_xyzw, lin_xyz, ang_xyz | Static tags via `tag_keys` |
| **BatteryStateTranslator** | `sensor_msgs/msg/BatteryState` | voltage, current, charge, capacity, percentage, temperature, status, health, technology, present | Static tags via `tag_keys` |
| **TfBasicTranslator** | TF Transforms | trans_xyz, rot_xyzw | Uses `custom_config` for source/target/parent frames and frequency |
| **TwistTranslator** | `geometry_msgs/msg/Twist` | linear_xyz, angular_xyz | Static tags via `tag_keys` |
| **BoolTranslator** | `std_msgs/msg/Bool` | value | Static tags via `tag_keys` |
| **NavSatFixTranslator** | `sensor_msgs/msg/NavSatFix` | latitude, longitude, altitude, status, service, cov_xx/yy/zz, cov_type | Covariance written only when type is not UNKNOWN |
| **JointStateTranslator** | `sensor_msgs/msg/JointState` | position, velocity, effort | One line per joint; joint name added as `joint` tag |

## File Structure

This package has been refactored to split the translator implementations into separate files for better maintainability:

### Common Utilities (in `robot_influx_bridge`)
- `robot_influx_bridge/include/robot_influx_bridge/translator_utils.hpp` - Header for common utility functions
- `robot_influx_bridge/src/translator_utils.cpp` - Implementation of `build_series_name` and `append_timestamp_if_valid`

### Individual Translators
- `src/imu_translator.cpp` - IMU message translator (`sensor_msgs/msg/Imu` → InfluxDB)
- `src/odometry_translator.cpp` - Odometry message translator (`nav_msgs/msg/Odometry` → InfluxDB)
- `src/battery_state_translator.cpp` - Battery state translator (`sensor_msgs/msg/BatteryState` → InfluxDB)
- `src/tf_basic_translator.cpp` - TF transform translator (periodic TF lookups → InfluxDB)
- `src/twist_translator.cpp` - Twist message translator (`geometry_msgs/msg/Twist` → InfluxDB)
- `src/bool_translator.cpp` - Bool message translator (`std_msgs/msg/Bool` → InfluxDB)
- `src/nav_sat_fix_translator.cpp` - GPS fix translator (`sensor_msgs/msg/NavSatFix` → InfluxDB)
- `src/joint_state_translator.cpp` - Joint state translator (`sensor_msgs/msg/JointState` → InfluxDB)

## Configuration Examples

### IMU Translator
```yaml
imu:
  topic: "/robot/imu/data"
  translator: "robot_influx_common_interfaces::ImuTranslator"
  measurement: "imu"
  max_rate: 200.0
  tag_keys:
    - "frame_id=imu_link"
```

### TF Translator (Transform Lookups)
```yaml
tf_map:
  topic: ""                       # Not used - TF lookups don't need topic
  translator: "robot_influx_common_interfaces::TfBasicTranslator"
  measurement: "tf"
  custom_config:
    - "source_frame=robot_base_link"
    - "target_frame=robot_odom"
    - "parent_frame=robot_odom"
    - "buffer_time_sec=10.0"
    - "lookup_timeout_sec=0.1"
    - "frequency_hz=50.0"
```

## Benefits of the Split:
1. **Maintainability**: Each translator is in its own file, making it easier to modify individual translators
2. **Modularity**: Common utilities are shared through a header file
3. **Build Performance**: Changes to one translator don't require recompiling all others
4. **Code Organization**: Related functionality is grouped together

All translators are still compiled into the same shared library (`robot_influx_common_interfaces_translators`) and use the same plugin registration system.
