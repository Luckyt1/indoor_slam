cmake_minimum_required(VERSION 3.16)
project(bxi_nav_interfaces)

# ============================================================================
# VENDORED COPY of the indoor-SLAM upstream interface package.
# Definitions are byte-for-byte the contract in docs/app-interface.md §7.
# When the real `bxi_nav_interfaces` (shipped by the indoor SLAM stack) becomes
# available on the robot + CI, DELETE this whole package — consumers
# (bxi_nav_gateway C++, bxi_bridge rclpy) import the same package name and need
# no change. Keeping both would collide on the package name in the workspace.
# Keep the vendored package ABI-compatible with the indoor-SLAM package even
# for interfaces the gateway does not call directly.  Both workspaces export
# the same package name, so omitting an action here can make a SLAM executable
# resolve this overlay's smaller typesupport library and fail at startup.
# ============================================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavPose.msg"
  "msg/NavStatus.msg"
  "msg/MappingStatus.msg"
  "msg/RuntimeStatus.msg"
  "msg/RelocalizationStatus.msg"
  "srv/SetInitialPose.srv"
  "srv/SaveMap.srv"
  "srv/ClearTerrain.srv"
  "srv/SetRuntimeMode.srv"
  "action/NavGoto.action"
  "action/BuildMap.action"
  "action/FollowWaypoints.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
