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
                # CPU 与协方差估计配置。上限 2: 机器人上 GICP 与网关/Nav2 共享
                # CPU, 4 线程会在 /initialpose 后的全图配准期饿死 WS 心跳
                # (App 表现为重定位确认后掉线)。2 线程 + 每周期省掉一次冗余
                # KdTree 构建后, 2Hz 配准仍有充足余量。
                "num_threads": 2,
                # 点协方差估计使用的最近邻数量。
                "num_neighbors": 10,
                # 降采样后的源点云点数低于该值时，跳过本次配准。
                "min_source_points": 200,
                # 常规跟踪最多迭代次数；配合较合理的终止阈值限制单周期耗时。
                "registration_max_iterations": 12,
                # 先验全局地图的体素降采样尺寸，单位：米。
                "global_leaf_size": 0.4,
                # 输入注册点云的体素降采样尺寸，单位：米；源云保持比先验图更细。
                "registered_leaf_size": 0.25,
                # 常规 GICP 收敛阈值：5mm / 0.5deg，避免为毫米级终止条件白耗迭代。
                "convergence_translation_epsilon": 0.005,
                "convergence_rotation_epsilon_deg": 0.5,
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
                # /initialpose 朝向假设数 (含给定朝向本身, 均匀铺满一圈):
                # App 端只点准位置、朝向给错时也能由粗配准内点率选出正确
                # 朝向。<=1 关闭搜索。4 个假设把最坏计算量限制为原来的一半。
                "initial_pose_yaw_hypotheses": 4,
                # 每个朝向假设的粗配准迭代上限 (只挑种子, 精配准由 2Hz 周期完成)。
                "initial_pose_search_iterations": 4,
                # 给定朝向已达到高质量门槛时提前结束，不再计算其余三个方向。
                "initial_pose_early_accept_inlier_ratio": 0.9,
                "initial_pose_early_accept_fitness": 0.5,
                # 粗搜索只在点击位置附近建临时目标树，避免每个朝向查询整张地图。
                # <=0 时使用完整目标地图。
                "initial_pose_search_radius": 15.0,
                # 退化检测 (长走廊): Hessian 平移块 λmin/λmax 低于该值时,
                # 抑制弱方向的平移更新分量, 交给里程计。<=0 关闭。
                "degeneracy_min_eigen_ratio": 0.05,
                # 为 true 时，收到 /initialpose 后才接受 GICP 校正更新。
                "require_initial_pose": True,
                # current_scan/aligned_scan 仅供 RViz 调试，生产环境关闭以避免
                # 每帧重复序列化点云和占用 DDS 带宽。
                "publish_debug_clouds": False,
                # 定位健康度 (/nav/reloc_required, true=需要重定位) 的超时:
                # 最近一次"被接受的 GICP 更新"距今超过该秒数即视为未定位。
                "localized_timeout": 10.0,
                # map->odom 按当前时刻 20Hz 重发并向未来覆盖 0.5s。定位扫描本身
                # 可能落后 0.2~0.35s，若沿用扫描时间戳，Nav2 Rotation Shim
                # 查询当前 TF 会持续 future extrapolation，背向目标无法起转。
                "transform_publish_tolerance": 0.5,
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
                "lidar_frame": "body_raw",
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
