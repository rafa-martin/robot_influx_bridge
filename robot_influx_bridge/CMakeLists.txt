cmake_minimum_required(VERSION 3.5)
project(robot_influx_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
ament_package_xml()

find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(CURL REQUIRED)
find_package(backward_ros REQUIRED)

# CORE
add_library(influx_bridge_lib
  src/influx_writer.cpp
)
target_include_directories(influx_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(influx_bridge_lib CURL::libcurl)


# ROS2 NODE
generate_parameter_library(
  bridge_parameters
  src/bridge_parameters.yaml
)

# TRANSLATORS
add_library(imu_translator SHARED src/translator/imu.cpp)
target_include_directories(imu_translator PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(imu_translator rclcpp sensor_msgs pluginlib)
target_link_libraries(imu_translator influx_bridge_lib)
pluginlib_export_plugin_description_file(robot_influx_bridge base_translator.xml)


add_library(influx_bridge SHARED
  src/influx_bridge.cpp
)
target_include_directories(influx_bridge PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(influx_bridge
  PUBLIC
    bridge_parameters
    influx_bridge_lib
    rclcpp::rclcpp
    pluginlib::pluginlib
    rclcpp_components::component)
rclcpp_components_register_node(influx_bridge
  PLUGIN "robot_influx_bridge::InfluxBridgeNode"
  EXECUTABLE influx_bridge_node
)

install(TARGETS influx_bridge_lib imu_translator influx_bridge bridge_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS influx_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components)

ament_package()
