// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "bxi_nav_interfaces/msg/relocalization_status.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // 启动时解析一次 base->lidar 先验地图变换 (随后热换图复用)。
  void resolvePriorMapTransform();
  // 加载/热换先验地图: 读 PCD → 变换 → 降采样 → 建树 → 估协方差, 全部成功
  // 后才原子换入成员; 同时把位姿状态重置为"等待 /initialpose"(与进程重启
  // 语义一致)。失败抛异常, 旧地图 (若有) 保持可用。
  void loadPriorMap(const std::string & file_name);
  // slam_manager 在 navigation→navigation 换图时调用的热换图服务
  // (nav2_msgs/srv/LoadMap, map_url = PCD 路径), 免去整进程重启和雷达断流。
  void onLoadMapService(
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);
  void performRegistration();
  void publishTransform();
  void publishGlobalMap();
  void publishRelocState();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  // /initialpose 多假设朝向搜索: 以给定位置为中心均匀撒 yaw 种子做粗配准,
  // 取内点率最优者作为种子位姿; 无缓存扫描或搜索失败时原样返回给定位姿。
  Eigen::Isometry3d searchYawHypotheses(
    const Eigen::Isometry3d & map_to_odom_guess, const Eigen::Isometry3d & map_to_robot_base,
    const Eigen::Isometry3d & odom_to_robot_base);
  pcl::PointCloud<pcl::PointCovariance>::Ptr cropCloudForSearch(
    const pcl::PointCloud<pcl::PointCovariance>::Ptr & cloud, double x, double y) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  int num_threads_;
  int num_neighbors_;
  int min_source_points_;
  int registration_max_iterations_;
  float global_leaf_size_;
  float registered_leaf_size_;
  double convergence_translation_epsilon_;
  double convergence_rotation_epsilon_deg_;
  float max_dist_sq_;
  double min_inlier_ratio_;
  double max_fitness_score_;
  double max_translation_update_;
  double max_rotation_update_;
  bool require_initial_pose_;
  bool publish_debug_clouds_;
  bool initial_pose_received_;
  bool has_global_map_msg_;
  // 定位健康度: 最近一次"被接受的 GICP 更新"距今超过该秒数即视为未定位。
  double localized_timeout_;
  // Future dating for map->odom, matching the standard localization TF
  // contract so consumers querying at "now" do not outrun delayed scans.
  double transform_publish_tolerance_;
  bool last_published_reloc_required_;
  rclcpp::Time last_accepted_time_;
  double last_fitness_score_;
  double last_inlier_ratio_;
  double last_translation_update_;
  double last_rotation_update_deg_;
  // /initialpose 朝向假设数 (含"按给定朝向"本身); <=1 关闭搜索。
  int initial_pose_yaw_hypotheses_;
  // 每个朝向假设的粗配准迭代上限 (一次性突发, 控制总耗时)。
  int initial_pose_search_iterations_;
  // 给定朝向达到高质量门槛时直接采用，避免继续计算其余朝向。
  double initial_pose_early_accept_inlier_ratio_;
  double initial_pose_early_accept_fitness_;
  // /initialpose 粗搜索只使用点击位置附近的目标点; <=0 使用完整目标地图。
  double initial_pose_search_radius_;
  // 退化检测: Hessian 平移块 λmin/λmax 低于该值时抑制弱方向平移分量; <=0 关闭。
  double degeneracy_min_eigen_ratio_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  std::string input_cloud_topic_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  // TF 定时器在独立 callback group 中并发读取最近一次有效校正。
  mutable std::mutex result_mutex_;
  // 启动时解析的先验地图坐标变换 (odom←lidar_odom); 热换图复用, 不再查 TF。
  Eigen::Affine3d prior_map_transform_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
  // 最近一帧协方差已就绪的源点云 —— /initialpose 朝向搜索直接复用,
  // 不必等下一个配准周期。
  pcl::PointCloud<pcl::PointCovariance>::Ptr last_source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  rclcpp::TimerBase::SharedPtr reloc_state_timer_;
  rclcpp::CallbackGroup::SharedPtr transform_callback_group_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reloc_required_pub_;
  rclcpp::Publisher<bxi_nav_interfaces::msg::RelocalizationStatus>::SharedPtr
    reloc_status_pub_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_srv_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
