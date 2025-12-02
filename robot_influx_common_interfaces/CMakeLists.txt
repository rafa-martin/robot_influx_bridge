cmake_minimum_required(VERSION 3.5)
project(robot_influx_common_interfaces)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_influx_bridge REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(robot_influx_common_interfaces_translators SHARED
  src/translator_utils.cpp
  src/imu_translator.cpp
  src/odometry_translator.cpp
  src/battery_state_translator.cpp
  src/tf_basic_translator.cpp
)
target_include_directories(robot_influx_common_interfaces_translators PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/>)
ament_target_dependencies(robot_influx_common_interfaces_translators
  pluginlib
  rclcpp
  sensor_msgs
  nav_msgs
  robot_influx_bridge
  builtin_interfaces
  geometry_msgs
  tf2
  tf2_ros)

pluginlib_export_plugin_description_file(robot_influx_bridge translators.xml)

install(TARGETS robot_influx_common_interfaces_translators
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES translators.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(pluginlib rclcpp sensor_msgs nav_msgs robot_influx_bridge builtin_interfaces geometry_msgs tf2 tf2_ros)

ament_package()
