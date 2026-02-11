cmake_minimum_required(VERSION 3.8)
project(ros2srrc_planning_request_adapters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Register under moveit_core so the planning pipeline's ClassLoader finds us (it uses
# pluginlib::ClassLoader<...>("moveit_core", "planning_request_adapter::PlanningRequestAdapter")).
pluginlib_export_plugin_description_file(moveit_core planning_request_adapters_plugin_description.xml)

add_library(ros2srrc_planning_request_adapters SHARED
  src/set_default_planner_id.cpp
)
target_include_directories(ros2srrc_planning_request_adapters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ros2srrc_planning_request_adapters
  moveit_core
  moveit_ros_planning
  pluginlib
  rclcpp
)

install(TARGETS ros2srrc_planning_request_adapters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
# Installed by pluginlib_export_plugin_description_file above

# So pluginlib can discover our adapter when move_group loads "ros2srrc_planning_request_adapters/SetDefaultPlannerId"
ament_export_include_directories(include)
ament_export_libraries(ros2srrc_planning_request_adapters)
ament_export_dependencies(moveit_core moveit_ros_planning pluginlib rclcpp)

ament_package()
