cmake_minimum_required(VERSION 3.8)
project(nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(indoor_nav_goal src/main.cpp)

ament_target_dependencies(indoor_nav_goal
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
)

install(TARGETS indoor_nav_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
