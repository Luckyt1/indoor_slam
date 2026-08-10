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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

namespace
{

Eigen::Isometry3d makePlanarTransform(double x, double y, double yaw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() << x, y, 0.0;
  transform.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return transform;
}

double planarYaw(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Isometry3d projectToPlanar(const Eigen::Isometry3d & transform)
{
  return makePlanarTransform(
    transform.translation().x(), transform.translation().y(), planarYaw(transform.rotation()));
}

}  // namespace

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  initial_pose_received_(false),
  has_global_map_msg_(false),
  last_published_reloc_required_(true),
  last_accepted_time_(0, 0, RCL_ROS_TIME),
  last_fitness_score_(0.0),
  last_inlier_ratio_(0.0),
  last_translation_update_(0.0),
  last_rotation_update_deg_(0.0),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("min_source_points", 200);
  this->declare_parameter("registration_max_iterations", 12);
  this->declare_parameter("global_leaf_size", 0.4);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("convergence_translation_epsilon", 0.005);
  this->declare_parameter("convergence_rotation_epsilon_deg", 0.5);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("min_inlier_ratio", 0.35);
  this->declare_parameter("max_fitness_score", 2.0);
  this->declare_parameter("max_translation_update", 1.0);
  this->declare_parameter("max_rotation_update_deg", 20.0);
  this->declare_parameter("initial_pose_yaw_hypotheses", 4);
  this->declare_parameter("initial_pose_search_iterations", 4);
  this->declare_parameter("initial_pose_early_accept_inlier_ratio", 0.9);
  this->declare_parameter("initial_pose_early_accept_fitness", 0.5);
  this->declare_parameter("initial_pose_search_radius", 15.0);
  this->declare_parameter("degeneracy_min_eigen_ratio", 0.05);
  this->declare_parameter("require_initial_pose", false);
  this->declare_parameter("publish_debug_clouds", false);
  this->declare_parameter("localized_timeout", 10.0);
  this->declare_parameter("transform_publish_tolerance", 0.5);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("registration_max_iterations", registration_max_iterations_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("convergence_translation_epsilon", convergence_translation_epsilon_);
  this->get_parameter("convergence_rotation_epsilon_deg", convergence_rotation_epsilon_deg_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("max_fitness_score", max_fitness_score_);
  this->get_parameter("max_translation_update", max_translation_update_);
  this->get_parameter("max_rotation_update_deg", max_rotation_update_);
  this->get_parameter("initial_pose_yaw_hypotheses", initial_pose_yaw_hypotheses_);
  this->get_parameter("initial_pose_search_iterations", initial_pose_search_iterations_);
  this->get_parameter(
    "initial_pose_early_accept_inlier_ratio", initial_pose_early_accept_inlier_ratio_);
  this->get_parameter("initial_pose_early_accept_fitness", initial_pose_early_accept_fitness_);
  this->get_parameter("initial_pose_search_radius", initial_pose_search_radius_);
  this->get_parameter("degeneracy_min_eigen_ratio", degeneracy_min_eigen_ratio_);
  this->get_parameter("require_initial_pose", require_initial_pose_);
  this->get_parameter("publish_debug_clouds", publish_debug_clouds_);
  this->get_parameter("localized_timeout", localized_timeout_);
  this->get_parameter("transform_publish_tolerance", transform_publish_tolerance_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  if (
    num_threads_ <= 0 || num_neighbors_ < 3 || min_source_points_ <= 0 ||
    registration_max_iterations_ < 2) {
    throw std::invalid_argument("GICP thread, neighbor and point limits must be positive");
  }
  if (
    !std::isfinite(global_leaf_size_) || global_leaf_size_ <= 0.0F ||
    !std::isfinite(registered_leaf_size_) || registered_leaf_size_ <= 0.0F) {
    throw std::invalid_argument("GICP voxel leaf sizes must be finite and positive");
  }
  if (
    !std::isfinite(convergence_translation_epsilon_) ||
    convergence_translation_epsilon_ <= 0.0 || convergence_translation_epsilon_ > 0.1 ||
    !std::isfinite(convergence_rotation_epsilon_deg_) ||
    convergence_rotation_epsilon_deg_ <= 0.0 || convergence_rotation_epsilon_deg_ > 5.0) {
    throw std::invalid_argument("GICP convergence tolerances are outside safe limits");
  }
  if (
    !std::isfinite(transform_publish_tolerance_) || transform_publish_tolerance_ < 0.0 ||
    transform_publish_tolerance_ > 2.0) {
    throw std::invalid_argument("transform_publish_tolerance must be finite and in [0, 2]");
  }
  if (
    initial_pose_yaw_hypotheses_ < 1 || initial_pose_search_iterations_ < 2 ||
    !std::isfinite(initial_pose_early_accept_inlier_ratio_) ||
    initial_pose_early_accept_inlier_ratio_ < min_inlier_ratio_ ||
    initial_pose_early_accept_inlier_ratio_ > 1.0 ||
    !std::isfinite(initial_pose_early_accept_fitness_) ||
    initial_pose_early_accept_fitness_ <= 0.0 ||
    !std::isfinite(initial_pose_search_radius_) || initial_pose_search_radius_ < 0.0) {
    throw std::invalid_argument("initial-pose search limits must be finite and positive");
  }

  max_rotation_update_ = max_rotation_update_ * std::acos(-1.0) / 180.0;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  register_->criteria.translation_eps = convergence_translation_epsilon_;
  register_->criteria.rotation_eps =
    convergence_rotation_epsilon_deg_ * std::acos(-1.0) / 180.0;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "relocalization/global_map", rclcpp::QoS(1).transient_local().reliable());
  if (publish_debug_clouds_) {
    current_scan_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("relocalization/current_scan", 2);
    aligned_scan_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("relocalization/aligned_scan", 2);
  }

  // 定位健康度信号: true=需要重定位。App 网关 (bxi_rc_ros2) 订阅后转发成
  // WS `nav.reloc_required`, App 以此为权威判定 (代替"收到 pose 流就算已定位")。
  // "已定位"标准 = 收到过 /initialpose 且最近 localized_timeout 秒内至少一次
  // "被接受的 GICP 更新" (converged + 内点率/fitness/步长全部过门槛)。
  // transient_local 让晚起的网关立即拿到当前值; 定时器 1Hz 周期重发, 保证
  // 中途重连的 WS 客户端也能在 1 秒内收敛。
  reloc_required_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/nav/reloc_required", rclcpp::QoS(1).reliable().transient_local());
  reloc_status_pub_ =
    this->create_publisher<bxi_nav_interfaces::msg::RelocalizationStatus>(
    "/nav/relocalization_status", rclcpp::QoS(1).reliable().transient_local());

  resolvePriorMapTransform();
  loadPriorMap(prior_pcd_file_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters。必须在 loadPriorMap
  // 之后应用: 加载会把位姿状态重置成"等待 /initialpose"。
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
    initial_pose_received_ = true;
  }
  previous_result_t_ = result_t_;

  // slam_manager 的 navigation→navigation 热换图入口 (见 onLoadMapService)。
  load_map_srv_ = this->create_service<nav2_msgs::srv::LoadMap>(
    "relocalization/load_map",
    std::bind(
      &SmallGicpRelocalizationNode::onLoadMapService, this, std::placeholders::_1,
      std::placeholders::_2));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS().keep_last(2),
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this),
    transform_callback_group_);

  reloc_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&SmallGicpRelocalizationNode::publishRelocState, this));
}

void SmallGicpRelocalizationNode::publishRelocState()
{
  // nanoseconds()==0 = 从未有过被接受的更新 (或刚被 /initialpose 重置),
  // 先判空再做减法, 避免与默认构造的 Time 混用时钟类型抛异常。
  const bool fix_fresh = last_accepted_time_.nanoseconds() != 0 &&
                         (this->now() - last_accepted_time_).seconds() < localized_timeout_;
  const bool required = !(initial_pose_received_ && fix_fresh);
  if (required != last_published_reloc_required_) {
    RCLCPP_INFO(
      this->get_logger(), "Relocalization state -> %s",
      required ? "RELOC REQUIRED" : "LOCALIZED");
    last_published_reloc_required_ = required;
  }
  std_msgs::msg::Bool msg;
  msg.data = required;
  reloc_required_pub_->publish(msg);

  bxi_nav_interfaces::msg::RelocalizationStatus status;
  status.header.stamp = this->now();
  status.header.frame_id = map_frame_;
  status.localized = !required;
  status.fitness_score = static_cast<float>(last_fitness_score_);
  status.inlier_ratio = static_cast<float>(last_inlier_ratio_);
  status.translation_update = static_cast<float>(last_translation_update_);
  status.rotation_update_deg = static_cast<float>(last_rotation_update_deg_);
  reloc_status_pub_->publish(status);
}

void SmallGicpRelocalizationNode::resolvePriorMapTransform()
{
  // Transform global pcd_map into the odom frame when a base/lidar offset is configured.
  prior_map_transform_ = Eigen::Affine3d::Identity();
  if (base_frame_.empty() || lidar_frame_.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "base_frame or lidar_frame is empty; using identity transform for the prior map");
    return;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      prior_map_transform_ = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << prior_map_transform_.translation().transpose() << ", rpy = "
                              << prior_map_transform_.rotation().eulerAngles(0, 1, 2).transpose());
      return;
    } catch (tf2::TransformException & ex) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error(
                "timed out resolving prior-map frame transform " + base_frame_ + " <- " +
                lidar_frame_ + ": " + ex.what());
      }
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
}

void SmallGicpRelocalizationNode::loadPriorMap(const std::string & file_name)
{
  if (file_name.empty()) {
    throw std::runtime_error("prior PCD path is empty");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *raw_map) == -1) {
    throw std::runtime_error("could not read prior PCD file: " + file_name);
  }
  if (raw_map->empty()) {
    throw std::runtime_error("prior PCD contains no points: " + file_name);
  }
  RCLCPP_INFO(
    this->get_logger(), "Loaded global map with %zu points from %s", raw_map->points.size(),
    file_name.c_str());

  pcl::transformPointCloud(*raw_map, *raw_map, prior_map_transform_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  auto target = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *raw_map, global_leaf_size_);

  const auto minimum_target_points = static_cast<size_t>(
    std::max(min_source_points_, num_neighbors_ + 1));
  if (!target || target->size() < minimum_target_points) {
    throw std::runtime_error(
            "prior PCD has too few usable points after downsampling: " +
            std::to_string(target ? target->size() : 0) + " < " +
            std::to_string(minimum_target_points));
  }

  // Build the target KdTree once, then reuse it for covariance estimation.
  // The tree-less estimate_covariances_omp overload would build a second,
  // throwaway KdTree over the same (potentially multi-million point) map.
  auto target_tree =
    std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target, small_gicp::KdTreeBuilderOMP(num_threads_));
  small_gicp::estimate_covariances_omp(*target, *target_tree, num_neighbors_, num_threads_);
  RCLCPP_INFO(
    this->get_logger(), "Prepared GICP target with %zu points at %.2f m leaf size",
    target->size(), static_cast<double>(global_leaf_size_));

  // 全部构建成功后才换入成员: 热换图中途失败时旧地图保持完好可用。
  // 全分辨率先验地图此后不再使用 (target 已采样, 展示消息已缓存到
  // global_map_msg_), raw_map 出栈即释放, 避免整张公司级地图常驻内存。
  pcl::toROSMsg(*raw_map, global_map_msg_);
  global_map_msg_.header.frame_id = map_frame_;
  has_global_map_msg_ = true;
  target_ = std::move(target);
  target_tree_ = std::move(target_tree);
  prior_pcd_file_ = file_name;

  // 新地图上旧的 map->odom 毫无意义。与进程重启后的语义保持一致: 回到
  // identity、等待新的 /initialpose, 定位健康度立即翻回"需要重定位"。
  previous_result_t_ = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_t_ = previous_result_t_;
  }
  initial_pose_received_ = false;
  last_accepted_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  publishGlobalMap();
  publishRelocState();
}

void SmallGicpRelocalizationNode::onLoadMapService(
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  // slam_manager 在 navigation→navigation 换图时调用: 进程不重启、雷达不断
  // 流, 原地换先验地图。地图加载与配准仍在默认互斥 callback group 中串行;
  // 独立 TF callback group 只读取受锁保护的最新校正快照。
  try {
    loadPriorMap(request->map_url);
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
    RCLCPP_INFO(
      this->get_logger(), "Hot-swapped prior map: %s", request->map_url.c_str());
  } catch (const std::exception & error) {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    RCLCPP_ERROR(
      this->get_logger(), "Prior map hot-swap failed (%s): %s", request->map_url.c_str(),
      error.what());
  }
}

void SmallGicpRelocalizationNode::publishGlobalMap()
{
  if (!has_global_map_msg_) {
    return;
  }

  global_map_msg_.header.stamp = this->now();
  global_map_pub_->publish(global_map_msg_);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*msg, *scan);

  if (publish_debug_clouds_) {
    current_scan_pub_->publish(*msg);
  }

  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  const bool waiting_for_initial_pose = require_initial_pose_ && !initial_pose_received_;
  if (waiting_for_initial_pose) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for /initialpose before accepting GICP updates.");
    // 不配准, 但继续走降采样+协方差把 last_source_ 备好: 首次 /initialpose
    // 到达时朝向假设搜索才有扫描可用 (否则最常见的"冷启动后第一次重定位"
    // 反而享受不到搜索)。
  }

  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  accumulated_cloud_->clear();

  if (source_->size() < static_cast<size_t>(std::max(min_source_points_, 0))) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Rejecting GICP input: too few source points after downsampling (%zu < %d).",
      source_->size(), min_source_points_);
    return;
  }

  // align() 只查询 target KdTree; source 侧仅协方差估计需要近邻,
  // 交给 estimate_covariances_omp 内部的一次性树, 不再重复显式建树。
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  // 协方差就绪的最近源云 —— /initialpose 的朝向假设搜索直接复用。
  last_source_ = source_;
  if (waiting_for_initial_pose) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  // 上限而非固定迭代数: 稳态跟踪一两次内收敛即提前退出, 放宽上限只帮助
  // /initialpose 之后初始误差较大的首次配准。
  register_->optimizer.max_iterations = registration_max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  // 退化方向检测 (LOAM 式解重映射): 长走廊等场景下 xy 平移的某个方向缺乏
  // 几何约束, 配准"收敛得很好"但弱方向的解纯属噪声, 会表现为沿走廊方向的
  // 定位漂移/横跳。Hessian 平移块与下方 delta 同为右乘扰动参数化 (机体系,
  // small_gicp 的 J 平移列为 -T.linear()), 最小/最大特征值之比低于阈值时,
  // 只保留平移增量在强方向上的投影, 弱方向交给里程计。
  if (result.converged && degeneracy_min_eigen_ratio_ > 0.0) {
    const Eigen::Matrix2d translation_hessian = result.H.block<2, 2>(3, 3);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(translation_hessian);
    const double weak_eigenvalue = eigen_solver.eigenvalues()(0);
    const double strong_eigenvalue = eigen_solver.eigenvalues()(1);
    if (
      strong_eigenvalue > 0.0 && std::isfinite(weak_eigenvalue) &&
      weak_eigenvalue / strong_eigenvalue < degeneracy_min_eigen_ratio_) {
      const Eigen::Isometry3d raw_delta = previous_result_t_.inverse() * result.T_target_source;
      const Eigen::Vector2d strong_direction = eigen_solver.eigenvectors().col(1);
      Eigen::Isometry3d projected_delta = raw_delta;
      projected_delta.translation().head<2>() =
        strong_direction * strong_direction.dot(raw_delta.translation().head<2>());
      result.T_target_source = previous_result_t_ * projected_delta;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Degenerate GICP geometry (eigen ratio %.4f < %.4f): weak-axis translation suppressed.",
        weak_eigenvalue / strong_eigenvalue, degeneracy_min_eigen_ratio_);
    }
  }

  const double inlier_ratio =
    source_->empty() ? 0.0 : static_cast<double>(result.num_inliers) / source_->size();
  const double fitness_score = result.num_inliers == 0
                                ? std::numeric_limits<double>::infinity()
                                : result.error / static_cast<double>(result.num_inliers);
  const Eigen::Isometry3d delta = previous_result_t_.inverse() * result.T_target_source;
  const double translation_update = delta.translation().norm();
  const double rotation_update = Eigen::AngleAxisd(delta.rotation()).angle();
  last_fitness_score_ = fitness_score;
  last_inlier_ratio_ = inlier_ratio;
  last_translation_update_ = translation_update;
  last_rotation_update_deg_ = rotation_update * 180.0 / std::acos(-1.0);

  if (!result.converged) {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
    return;
  }

  if (
    !std::isfinite(fitness_score) || inlier_ratio < min_inlier_ratio_ ||
    fitness_score > max_fitness_score_ || translation_update > max_translation_update_ ||
    rotation_update > max_rotation_update_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Rejected GICP update: inliers=%zu/%zu ratio=%.3f fitness=%.3f d_trans=%.3f "
      "d_rot=%.1fdeg",
      result.num_inliers, source_->size(), inlier_ratio, fitness_score, translation_update,
      rotation_update * 180.0 / std::acos(-1.0));
    return;
  }

  previous_result_t_ = result.T_target_source;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_t_ = previous_result_t_;
  }
  // "被接受的更新"即定位成功的权威证据 (匹配点数/内点率已过门槛),
  // 刷新时间戳并立即广播, App 端重定位确认后 ~1 个配准周期内就能看到"已定位"。
  last_accepted_time_ = this->now();
  publishRelocState();

  if (publish_debug_clouds_) {
    pcl::PointCloud<pcl::PointXYZ> source_xyz;
    source_xyz.reserve(source_->size());
    for (const auto & point : source_->points) {
      source_xyz.emplace_back(point.x, point.y, point.z);
    }

    pcl::PointCloud<pcl::PointXYZ> aligned_scan;
    pcl::transformPointCloud(source_xyz, aligned_scan, result.T_target_source.matrix());

    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned_scan, aligned_msg);
    aligned_msg.header.stamp = last_scan_time_.nanoseconds() == 0 ? this->now() : last_scan_time_;
    aligned_msg.header.frame_id = map_frame_;
    aligned_scan_pub_->publish(aligned_msg);
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Accepted GICP update: inliers=%zu/%zu ratio=%.3f fitness=%.3f d_trans=%.3f "
    "d_rot=%.1fdeg",
    result.num_inliers, source_->size(), inlier_ratio, fitness_score, translation_update,
    rotation_update * 180.0 / std::acos(-1.0));
}

void SmallGicpRelocalizationNode::publishTransform()
{
  Eigen::Isometry3d result_snapshot;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_snapshot = result_t_;
  }
  if (result_snapshot.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // result_t_ is the slowly corrected map->odom transform, while odom->base
  // continues updating at sensor rate. Re-publish the latest correction at
  // wall/ROS "now" and future-date it like AMCL. Stamping it with the delayed
  // input scan made Nav2 Rotation Shim's lookup at now fail continuously with
  // "extrapolation into the future", so required turns were silently skipped.
  transform_stamped.header.stamp = this->now() +
    rclcpp::Duration::from_seconds(transform_publish_tolerance_);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_snapshot.translation();
  const Eigen::Quaterniond rotation(result_snapshot.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  const auto & pose = msg->pose.pose;
  if (
    !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
    !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
    !std::isfinite(pose.orientation.w)) {
    RCLCPP_ERROR(this->get_logger(), "Rejecting initial pose with non-finite values");
    return;
  }
  Eigen::Quaterniond map_to_robot_base_rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const double quaternion_norm = map_to_robot_base_rotation.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-6) {
    RCLCPP_ERROR(this->get_logger(), "Rejecting initial pose with invalid quaternion");
    return;
  }
  map_to_robot_base_rotation.normalize();
  const Eigen::Isometry3d map_to_robot_base = makePlanarTransform(
    pose.position.x, pose.position.y,
    planarYaw(map_to_robot_base_rotation.toRotationMatrix()));

  try {
    auto transform =
      tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, tf2::TimePointZero);
    Eigen::Isometry3d odom_to_robot_base = projectToPlanar(
      tf2::transformToEigen(transform.transform));
    Eigen::Isometry3d map_to_odom = map_to_robot_base * odom_to_robot_base.inverse();
    // 多假设朝向搜索: 用户在 App 里只需要点准位置, 朝向给错也能被纠正
    // (最优假设由粗配准内点率选出); 搜索失败/无缓存扫描时按给定朝向种子。
    map_to_odom = searchYawHypotheses(map_to_odom, map_to_robot_base, odom_to_robot_base);

    initial_pose_received_ = true;
    previous_result_t_ = map_to_odom;
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      result_t_ = previous_result_t_;
    }
    // 重置定位健康度: 用户刚设的位姿还没被扫描匹配验证过, 先回到"未定位",
    // 等下一次被接受的 GICP 更新 (通常 <1s) 再翻成"已定位" — 匹配不上就一直
    // 保持"需要重定位", App 端能如实看到这次重定位没有成功。
    last_accepted_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publishRelocState();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      odom_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
  }
}

pcl::PointCloud<pcl::PointCovariance>::Ptr SmallGicpRelocalizationNode::cropCloudForSearch(
  const pcl::PointCloud<pcl::PointCovariance>::Ptr & cloud, double x, double y) const
{
  if (!cloud || initial_pose_search_radius_ <= 0.0) {
    return cloud;
  }

  auto cropped = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
  cropped->reserve(cloud->size());
  const double radius_sq = initial_pose_search_radius_ * initial_pose_search_radius_;
  for (const auto & point : cloud->points) {
    const double dx = static_cast<double>(point.x) - x;
    const double dy = static_cast<double>(point.y) - y;
    if (dx * dx + dy * dy <= radius_sq) {
      cropped->push_back(point);
    }
  }

  return cropped;
}

Eigen::Isometry3d SmallGicpRelocalizationNode::searchYawHypotheses(
  const Eigen::Isometry3d & map_to_odom_guess, const Eigen::Isometry3d & map_to_robot_base,
  const Eigen::Isometry3d & odom_to_robot_base)
{
  if (initial_pose_yaw_hypotheses_ <= 1) {
    return map_to_odom_guess;
  }
  const auto minimum_points = static_cast<size_t>(std::max(min_source_points_, 0));
  if (!last_source_ || last_source_->size() < minimum_points) {
    RCLCPP_INFO(
      this->get_logger(), "Yaw hypothesis search skipped: no cached scan yet, seeding as given.");
    return map_to_odom_guess;
  }

  const double base_x = map_to_robot_base.translation().x();
  const double base_y = map_to_robot_base.translation().y();
  const double base_yaw = planarYaw(map_to_robot_base.rotation());
  constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

  const auto search_started_at = std::chrono::steady_clock::now();
  const auto search_target = cropCloudForSearch(target_, base_x, base_y);
  const auto search_source = cropCloudForSearch(
    last_source_, odom_to_robot_base.translation().x(), odom_to_robot_base.translation().y());
  const auto minimum_target_points = static_cast<size_t>(
    std::max(min_source_points_, num_neighbors_ + 1));
  if (
    !search_target || search_target->size() < minimum_target_points || !search_source ||
    search_source->size() < minimum_points) {
    RCLCPP_WARN(
      this->get_logger(),
      "Yaw hypothesis search skipped: local crop has %zu target and %zu source points.",
      search_target ? search_target->size() : 0, search_source ? search_source->size() : 0);
    return map_to_odom_guess;
  }
  auto search_tree = target_tree_;
  if (search_target != target_) {
    search_tree =
      std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      search_target, small_gicp::KdTreeBuilderOMP(num_threads_));
  }

  // 一次性突发: 每个假设一轮少迭代的粗配准, 结果只用来挑种子; 精配准仍由
  // 2Hz 周期完成。performRegistration 每周期都会重设迭代上限, 这里改完
  // 不必恢复。
  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = std::max(2, initial_pose_search_iterations_);

  double best_inlier_ratio = -1.0;
  double max_inlier_ratio_seen = -1.0;
  double best_fitness = std::numeric_limits<double>::infinity();
  int best_index = -1;
  Eigen::Isometry3d best_transform = map_to_odom_guess;
  for (int hypothesis = 0; hypothesis < initial_pose_yaw_hypotheses_; ++hypothesis) {
    // hypothesis 0 = 按给定朝向, 其余均匀铺满一圈。
    const double yaw_offset =
      kTwoPi * static_cast<double>(hypothesis) / static_cast<double>(initial_pose_yaw_hypotheses_);
    const Eigen::Isometry3d seed =
      makePlanarTransform(base_x, base_y, base_yaw + yaw_offset) * odom_to_robot_base.inverse();
    const auto result = register_->align(*search_target, *search_source, *search_tree, seed);
    if (
      result.num_inliers == 0 || !std::isfinite(result.error) ||
      !result.T_target_source.matrix().allFinite()) {
      RCLCPP_DEBUG(
        this->get_logger(), "Yaw hypothesis %d/%d unusable: converged=%s inliers=%zu",
        hypothesis, initial_pose_yaw_hypotheses_, result.converged ? "true" : "false",
        result.num_inliers);
      continue;
    }
    const double inlier_ratio =
      static_cast<double>(result.num_inliers) / static_cast<double>(search_source->size());
    const double fitness = result.error / static_cast<double>(result.num_inliers);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Yaw hypothesis %d/%d: converged=%s inliers=%zu ratio=%.3f fitness=%.3f",
      hypothesis, initial_pose_yaw_hypotheses_, result.converged ? "true" : "false",
      result.num_inliers, inlier_ratio, fitness);
    if (inlier_ratio < min_inlier_ratio_ || fitness > max_fitness_score_) {
      continue;
    }

    if (
      hypothesis == 0 && inlier_ratio >= initial_pose_early_accept_inlier_ratio_ &&
      fitness <= initial_pose_early_accept_fitness_) {
      best_inlier_ratio = inlier_ratio;
      best_fitness = fitness;
      best_index = hypothesis;
      best_transform = result.T_target_source;
      break;
    }

    // 粗搜索不要求达到 small_gicp 的毫米级终止阈值; 它只选择有限、过质量门槛
    // 的种子，最终 LOCALIZED 仍必须由后续严格收敛的常规 GICP 更新确认。
    const bool better = best_index < 0 || inlier_ratio > max_inlier_ratio_seen + 0.02 ||
      (inlier_ratio >= max_inlier_ratio_seen - 0.02 && fitness < best_fitness);
    max_inlier_ratio_seen = std::max(max_inlier_ratio_seen, inlier_ratio);
    if (better) {
      best_inlier_ratio = inlier_ratio;
      best_fitness = fitness;
      best_index = hypothesis;
      best_transform = result.T_target_source;
    }
  }

  if (best_index < 0 || best_inlier_ratio < min_inlier_ratio_) {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - search_started_at).count();
    RCLCPP_WARN(
      this->get_logger(),
      "Yaw hypothesis search found no usable seed in %.1f ms using %zu target points; "
      "seeding as given.", elapsed_ms, search_target->size());
    return map_to_odom_guess;
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - search_started_at).count();
  RCLCPP_INFO(
    this->get_logger(),
    "Yaw hypothesis search completed in %.1f ms with %zu target points: hypothesis %d/%d "
    "wins (offset %.0f deg, inliers %.3f, fitness %.3f).",
    elapsed_ms, search_target->size(),
    best_index, initial_pose_yaw_hypotheses_,
    360.0 * static_cast<double>(best_index) / static_cast<double>(initial_pose_yaw_hypotheses_),
    best_inlier_ratio, best_fitness);
  return best_transform;
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
