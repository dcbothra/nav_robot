cmake_minimum_required(VERSION 3.8)
project(nav_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

find_package(nav2_behavior_tree REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(battery_sim_node scripts/battery_state.cpp)
ament_target_dependencies(battery_sim_node
  rclcpp sensor_msgs nav_msgs geometry_msgs)

add_library(bt_nodes SHARED behaviour_tree/bt_nodes.cpp)

ament_target_dependencies(bt_nodes
  rclcpp
  nav2_behavior_tree
  yaml-cpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  ament_index_cpp)

pluginlib_export_plugin_description_file(nav2_behavior_tree behaviour_tree/bt_plugins.xml)

ament_export_libraries(bt_nodes)
ament_export_dependencies(nav2_behavior_tree)

set_target_properties(bt_nodes PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

target_link_libraries(bt_nodes yaml-cpp)

install(TARGETS
  battery_sim_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  behaviour_tree/bt_plugins.xml
  DESTINATION share/${PROJECT_NAME}/behaviour_tree)

install(DIRECTORY
  urdf launch worlds maps config behaviour_tree
  DESTINATION share/${PROJECT_NAME})

ament_package()