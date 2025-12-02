# Translators Overview

This package provides a collection of reusable translators for common ROS 2 message types. Each translator converts ROS 2 messages into InfluxDB line protocol format for telemetry storage and analysis.

## Available Translators

| Translator | Message Type | InfluxDB Fields | Configuration Notes |
|------------|--------------|-----------------|-------------------|
| **ImuTranslator** | `sensor_msgs/msg/Imu` | orientation (quaternion), angular_velocity (xyz), linear_acceleration (xyz) | Supports frame_id as tag |
| **OdometryTranslator** | `nav_msgs/msg/Odometry` | pose position/orientation, twist linear/angular | Supports child_frame_id as tag |
| **BatteryStateTranslator** | `sensor_msgs/msg/BatteryState` | voltage, current, charge, capacity, percentage, power_supply_status | Multiple batteries supported |
| **TfBasicTranslator** | TF Transforms | position (xyz), orientation (quaternion) | Uses custom_config for transform parameters |

## Files Structure

This package has been refactored to split the translator implementations into separate files for better maintainability:

### Common Utilities
- `include/robot_influx_common_interfaces/translator_utils.hpp` - Header for common utility functions
- `src/translator_utils.cpp` - Implementation of common utility functions (`buildSeriesName`, `appendTimestampIfValid`)

### Individual Translators
- `src/imu_translator.cpp` - IMU message translator (`sensor_msgs/msg/Imu` → InfluxDB)
- `src/odometry_translator.cpp` - Odometry message translator (`nav_msgs/msg/Odometry` → InfluxDB) 
- `src/battery_state_translator.cpp` - Battery state translator (`sensor_msgs/msg/BatteryState` → InfluxDB)
- `src/tf_basic_translator.cpp` - TF transform translator (periodic TF lookups → InfluxDB)

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
