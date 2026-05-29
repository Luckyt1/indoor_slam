cmake_minimum_required(VERSION 3.8)
project(nav)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(indoor_nav_goal src/main.cpp)
add_executable(terrain_analysis src/terrain_analysis.cpp)

ament_target_dependencies(indoor_nav_goal
  nav_msgs
  rclcpp
  sensor_msgs
)

ament_target_dependencies(terrain_analysis
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  indoor_nav_goal
  terrain_analysis
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

set(INDOOR_SLAM_MAPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../maps")
if(EXISTS "${INDOOR_SLAM_MAPS_DIR}")
  install(DIRECTORY "${INDOOR_SLAM_MAPS_DIR}/"
    DESTINATION share/${PROJECT_NAME}/maps
    FILES_MATCHING
      PATTERN "*.yaml"
      PATTERN "*.pgm"
  )
endif()

ament_package()
