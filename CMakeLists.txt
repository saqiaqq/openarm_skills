cmake_minimum_required(VERSION 3.16)
project(openarm_skills)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ----------------------------------------------------------------------------
# Interface generation
# ----------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  "srv/Stop.srv"
  "srv/GotoHome.srv"
  "srv/Gripper.srv"
  "srv/StringCommand.srv"
  "srv/DetectGraspPose.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# ----------------------------------------------------------------------------
# Skill server executable
# ----------------------------------------------------------------------------
add_executable(skill_server_node
  src/skill_server_node.cpp
)

target_include_directories(skill_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(skill_server_node
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  control_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(skill_server_node "${cpp_typesupport_target}")

install(TARGETS skill_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
