cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

# Link directories (per your standing request)
link_directories(/opt/homebrew/opt/box2d/lib /opt/homebrew/opt/glfw/lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces (matches file: action/Waypoint.action)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ===== Server node =====
add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_geometry_msgs
  std_msgs rosgraph_msgs action_msgs unique_identifier_msgs builtin_interfaces
)
# Ensure generated interfaces are visible to this target
rosidl_target_interfaces(fastbot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executable so `ros2 run` can find it
install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME})

# ===== Tests =====
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_waypoints test/test_waypoints.cpp)
if(TARGET test_waypoints)
  ament_target_dependencies(test_waypoints
    rclcpp rclcpp_action geometry_msgs nav_msgs
    std_msgs rosgraph_msgs action_msgs unique_identifier_msgs builtin_interfaces
  )
  rosidl_target_interfaces(test_waypoints ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()