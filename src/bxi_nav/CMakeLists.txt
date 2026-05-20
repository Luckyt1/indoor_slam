cmake_minimum_required(VERSION 3.8)
project(nav)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(indoor_nav_goal src/main.cpp)

ament_target_dependencies(indoor_nav_goal
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  indoor_nav_goal
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
