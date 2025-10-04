cmake_minimum_required(VERSION 3.5)
project(robot_influx_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)

find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(CURL REQUIRED)
find_package(ZLIB REQUIRED)

# CORE
add_library(influx_bridge_lib
  src/influx_writer.cpp
  src/persistent_batch_store.cpp
)
target_include_directories(influx_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(influx_bridge_lib PUBLIC CURL::libcurl rclcpp::rclcpp ZLIB::ZLIB)

# ROS2 NODE
generate_parameter_library(
  bridge_parameters
  src/bridge_parameters.yaml
)

add_library(influx_bridge SHARED
  src/influx_error.cpp
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

install(TARGETS influx_bridge_lib influx_bridge bridge_parameters
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

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(pluginlib rclcpp rclcpp_components CURL ZLIB)

ament_package()
