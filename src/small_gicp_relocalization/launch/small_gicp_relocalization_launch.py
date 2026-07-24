# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=[
            {
                # CPU 与协方差估计配置；多核 CPU 上适当增大可提升预处理和 GICP 速度。
                "num_threads": 4,
                # 点协方差估计使用的最近邻数量。
                "num_neighbors": 10,
                # 降采样后的源点云点数低于该值时，跳过本次配准。
                "min_source_points": 200,
                # 先验全局地图的体素降采样尺寸，单位：米。
                "global_leaf_size": 0.25,
                # 输入注册点云的体素降采样尺寸，单位：米。
                "registered_leaf_size": 0.25,
                # GICP 接受的最大匹配点平方距离。
                "max_dist_sq": 1.0,
                # 内点比例低于该值时，拒绝本次 GICP 更新。
                "min_inlier_ratio": 0.35,
                # 平均内点误差高于该值时，拒绝本次 GICP 更新。
                "max_fitness_score": 2.0,
                # map->odom 平移更新量超过该值时拒绝更新，单位：米。
                "max_translation_update": 1.0,
                # map->odom 旋转更新量超过该值时拒绝更新，单位：度。
                "max_rotation_update_deg": 20.0,
                # 为 true 时，收到 /initialpose 后才接受 GICP 校正更新。
                "require_initial_pose": True,
                # 本节点发布校正 TF 的父坐标系。
                "map_frame": "map",
                # 本节点发布校正 TF 的子坐标系。
                "odom_frame": "odom",
                # 与 lidar_frame 配合，用于把先验地图转换到 odom 对齐空间。
                # 留空时不做额外变换，按单位变换处理先验地图。
                "base_frame": "",
                # 机器人本体坐标系，用于把 /initialpose 转换成 map->odom 校正。
                "robot_base_frame": "base_link",
                # 雷达坐标系，与 base_frame 配合做先验地图坐标修正。
                "lidar_frame": "base_raw",
                # 先验全局点云地图 PCD 文件路径。
                "prior_pcd_file": prior_pcd_file,
                # 输入 PointCloud2 话题，通常为 odom 空间下的注册点云。
                "input_cloud_topic": "cloud_registered",
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value="maps/PCD/scans.pcd",
            description="用于重定位和可视化发布的 PCD 先验地图文件",
        ),
        node,
    ])
