cmake_minimum_required(VERSION 3.16)
project(point_lio)
# 设置策略以消除警告
cmake_policy(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
add_compile_options(
    -std=c++17
    -pthread
    -fexceptions
)

# 优化设置
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# 多处理器设置（简化）
add_definitions(-DMP_PROC_NUM=1)

# ROS2 包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common io filters)

# 查找 Python 3 - 需要 Python.h
find_package(Python3 COMPONENTS Development REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

# 可执行文件
add_executable(pointlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
)

# ROS2 依赖
ament_target_dependencies(pointlio_mapping
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

# 准备链接库列表
set(ALL_LIBS
  ${PCL_LIBRARIES}
  Python3::Python
)

# 一次性链接所有库（使用普通签名）
target_link_libraries(pointlio_mapping ${ALL_LIBS})

# 安装
install(TARGETS
  pointlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
)

ament_package()
