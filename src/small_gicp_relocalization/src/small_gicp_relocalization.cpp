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
#include <cmath>
#include <limits>

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
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("min_source_points", 200);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("min_inlier_ratio", 0.35);
  this->declare_parameter("max_fitness_score", 2.0);
  this->declare_parameter("max_translation_update", 1.0);
  this->declare_parameter("max_rotation_update_deg", 20.0);
  this->declare_parameter("require_initial_pose", false);
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
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("max_fitness_score", max_fitness_score_);
  this->get_parameter("max_translation_update", max_translation_update_);
  this->get_parameter("max_rotation_update_deg", max_rotation_update_);
  this->get_parameter("require_initial_pose", require_initial_pose_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
    initial_pose_received_ = true;
  }
  max_rotation_update_ = max_rotation_update_ * std::acos(-1.0) / 180.0;
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "relocalization/global_map", rclcpp::QoS(1).transient_local().reliable());
  current_scan_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("relocalization/current_scan", 10);
  aligned_scan_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("relocalization/aligned_scan", 10);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));

  global_map_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&SmallGicpRelocalizationNode::publishGlobalMap, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // Transform global pcd_map into the odom frame when a base/lidar offset is configured.
  Eigen::Affine3d odom_to_lidar_odom = Eigen::Affine3d::Identity();
  if (base_frame_.empty() || lidar_frame_.empty()) {
    RCLCPP_INFO(
      this->get_logger(),
      "base_frame or lidar_frame is empty; using identity transform for the prior map");
  } else {
    while (true) {
      try {
        auto tf_stamped = tf_buffer_->lookupTransform(
          base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
        odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
        RCLCPP_INFO_STREAM(
          this->get_logger(), "odom_to_lidar_odom: translation = "
                                << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                                << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
        break;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
  pcl::toROSMsg(*global_map_, global_map_msg_);
  global_map_msg_.header.frame_id = map_frame_;
  has_global_map_msg_ = true;
  publishGlobalMap();
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

  sensor_msgs::msg::PointCloud2 debug_msg;
  pcl::toROSMsg(*scan, debug_msg);
  debug_msg.header = msg->header;
  current_scan_pub_->publish(debug_msg);

  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (require_initial_pose_ && !initial_pose_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for /initialpose before accepting GICP updates.");
    accumulated_cloud_->clear();
    return;
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

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  const double inlier_ratio =
    source_->empty() ? 0.0 : static_cast<double>(result.num_inliers) / source_->size();
  const double fitness_score = result.num_inliers == 0
                                ? std::numeric_limits<double>::infinity()
                                : result.error / static_cast<double>(result.num_inliers);
  const Eigen::Isometry3d delta = previous_result_t_.inverse() * result.T_target_source;
  const double translation_update = delta.translation().norm();
  const double rotation_update = Eigen::AngleAxisd(delta.rotation()).angle();

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

  result_t_ = previous_result_t_ = result.T_target_source;

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

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Accepted GICP update: inliers=%zu/%zu ratio=%.3f fitness=%.3f d_trans=%.3f "
    "d_rot=%.1fdeg",
    result.num_inliers, source_->size(), inlier_ratio, fitness_score, translation_update,
    rotation_update * 180.0 / std::acos(-1.0));
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  const rclcpp::Time stamp = last_scan_time_.nanoseconds() == 0 ? this->now() : last_scan_time_;
  transform_stamped.header.stamp = stamp + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

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

  const Eigen::Quaterniond map_to_robot_base_rotation(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  const Eigen::Isometry3d map_to_robot_base = makePlanarTransform(
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    planarYaw(map_to_robot_base_rotation.toRotationMatrix()));

  try {
    auto transform =
      tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, tf2::TimePointZero);
    Eigen::Isometry3d odom_to_robot_base = projectToPlanar(
      tf2::transformToEigen(transform.transform));
    Eigen::Isometry3d map_to_odom = map_to_robot_base * odom_to_robot_base.inverse();

    initial_pose_received_ = true;
    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      odom_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
