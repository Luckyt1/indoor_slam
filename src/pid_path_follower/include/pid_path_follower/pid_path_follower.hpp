#ifndef PID_PATH_FOLLOWER__PID_PATH_FOLLOWER_HPP_
#define PID_PATH_FOLLOWER__PID_PATH_FOLLOWER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <istream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"

namespace pid_path_follower
{

class PidPathFollower : public nav2_core::Controller
{
public:
  PidPathFollower() = default;
  ~PidPathFollower() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct PidState
  {
    double integral{0.0};
    double previous_error{0.0};
    bool has_previous_error{false};
  };

  struct LocalPoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct PathPoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct CollisionResult
  {
    bool has_cloud{false};
    bool stale_cloud{false};
    bool blocked{false};
    bool footprint_blocked{false};
    double nearest_distance{0.0};
    int point_count{0};
  };

  struct LocalPathSelection
  {
    bool found{false};
    bool blocked{false};
    double path_scale{1.0};
    int rotation_index{-1};
    int group_index{-1};
    std::vector<std::pair<double, double>> local_path;
  };

  void declareParameters();
  void loadPathLibrary();
  int readPlyVertexCount(std::istream & stream) const;
  void resetPid();
  double computePid(
    double error,
    double kp,
    double ki,
    double kd,
    double integral_limit,
    double dt,
    PidState & state);
  double limitRate(double desired, double previous, double limit, double dt) const;
  double activeMaxLinearVelocity() const;
  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation) const;
  double normalizeAngle(double angle) const;
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame) const;
  geometry_msgs::msg::PoseStamped getLookaheadPose(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    geometry_msgs::msg::PoseStamped & final_pose) const;
  geometry_msgs::msg::TwistStamped zeroCommand(
    const geometry_msgs::msg::PoseStamped & pose) const;
  void obstacleCloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  std::vector<LocalPoint> getObstaclePointsInRobotFrame(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    bool & has_cloud,
    bool & stale_cloud) const;
  std::vector<LocalPoint> getLocalCostmapObstaclePointsInRobotFrame(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  std::vector<std::pair<double, double>> buildLocalPath(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & target_pose) const;
  CollisionResult evaluatePathCollision(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::vector<LocalPoint> & obstacle_points) const;
  LocalPathSelection selectLibraryPath(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & global_target_pose,
    const geometry_msgs::msg::PoseStamped & final_pose,
    double speed_scale,
    const std::vector<LocalPoint> & obstacle_points);
  LocalPathSelection planLocalCostmapPath(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & fallback_goal);
  geometry_msgs::msg::PoseStamped getLocalCostmapGoal(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & fallback_goal) const;
  bool isCostmapCellBlocked(unsigned int mx, unsigned int my) const;
  bool directionAllowed(double desired_direction_deg, int rotation_index) const;
  void publishFreePaths(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double desired_direction_deg,
    double path_scale,
    double path_range,
    double relative_goal_distance,
    const std::vector<int> & clear_path_list,
    double min_obs_ang_cw,
    double min_obs_ang_ccw);
  void publishEmptyFreePaths(const geometry_msgs::msg::PoseStamped & robot_pose);
  void publishLocalCostmapPath(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const LocalPathSelection & selection);
  bool shouldPublishFreePaths();
  bool shouldUpdateLocalPath(const rclcpp::Time & now) const;
  geometry_msgs::msg::PoseStamped targetPoseFromLocalPath(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const LocalPathSelection & selection) const;
  double distanceToSegment(
    double px,
    double py,
    double ax,
    double ay,
    double bx,
    double by,
    double & segment_fraction) const;
  void warnCollisionThrottled(const CollisionResult & collision);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;

  double lookahead_distance_{0.6};
  double prune_distance_{0.35};
  double transform_tolerance_{0.2};
  double linear_kp_{0.8};
  double linear_ki_{0.0};
  double linear_kd_{0.05};
  double angular_kp_{2.0};
  double angular_ki_{0.0};
  double angular_kd_{0.05};
  double linear_integral_limit_{0.5};
  double angular_integral_limit_{1.0};
  double max_linear_velocity_{0.26};
  double min_linear_velocity_{0.0};
  double max_angular_velocity_{1.0};
  double max_linear_accel_{1.0};
  double max_angular_accel_{3.2};
  double goal_slowdown_radius_{0.8};
  double rotate_to_goal_distance_{0.4};
  double rotate_to_heading_threshold_{0.8};

  bool collision_enabled_{true};
  std::string collision_cloud_topic_{"/livox_points_odom"};
  double collision_cloud_timeout_{1.0};
  bool stop_on_stale_cloud_{false};
  double collision_voxel_size_{0.05};
  double collision_adjacent_range_{3.0};
  double collision_check_distance_{1.2};
  double collision_slow_distance_{1.2};
  double collision_stop_distance_{0.45};
  double collision_min_speed_scale_{0.2};
  double collision_lateral_margin_{0.08};
  double collision_min_z_{-0.5};
  double collision_max_z_{0.8};
  double collision_rear_margin_{0.1};
  double vehicle_length_{0.45};
  double vehicle_width_{0.55};
  int collision_point_threshold_{2};

  std::string path_folder_;
  bool use_path_library_{true};
  bool path_library_loaded_{false};
  bool use_local_costmap_planner_{true};
  bool use_local_costmap_obstacles_{true};
  bool fallback_to_local_costmap_on_no_free_path_{true};
  bool local_costmap_allow_unknown_{false};
  bool publish_local_costmap_path_{true};
  bool two_way_drive_{false};
  bool check_rot_obstacle_{true};
  bool path_scale_by_speed_{true};
  bool path_range_by_speed_{true};
  bool path_crop_by_goal_{false};
  bool dir_to_vehicle_{true};
  double path_scale_{1.5};
  double min_path_scale_{1.0};
  double path_scale_step_{0.1};
  double min_path_range_{1.0};
  double path_range_step_{0.5};
  double dir_weight_{0.2};
  double dir_threshold_deg_{120.0};
  double goal_clear_range_{0.5};
  double local_costmap_goal_distance_{1.5};
  double local_costmap_cost_weight_{3.0};
  double no_free_path_fallback_delay_{2.0};
  double fallback_min_duration_{3.0};
  int local_costmap_lethal_cost_{253};
  std::string local_costmap_path_topic_{"/local_plan"};
  double grid_voxel_size_{0.02};
  double search_radius_{0.45};
  double grid_voxel_offset_x_{3.2};
  double grid_voxel_offset_y_{4.5};
  bool visualize_free_paths_{true};
  std::string free_paths_topic_{"/free_paths"};
  double free_paths_publish_period_{0.2};
  double local_path_update_period_{0.2};
  int candidate_rotation_step_{1};
  int free_paths_rotation_step_{3};
  int free_paths_point_skip_{30};
  std::vector<std::vector<PathPoint>> start_paths_;
  std::vector<std::vector<PathPoint>> visual_paths_;
  std::vector<int> path_group_ids_;
  std::vector<double> path_end_direction_deg_;
  std::vector<std::vector<int>> correspondences_;

  double speed_limit_{0.0};
  bool has_speed_limit_{false};
  PidState linear_pid_;
  PidState angular_pid_;
  double previous_linear_cmd_{0.0};
  double previous_angular_cmd_{0.0};
  rclcpp::Time previous_time_;
  bool has_previous_time_{false};
  LocalPathSelection cached_selection_;
  CollisionResult cached_collision_;
  rclcpp::Time last_local_path_update_time_;
  bool has_cached_selection_{false};
  rclcpp::Time no_free_path_start_time_;
  rclcpp::Time fallback_start_time_;
  bool no_free_path_timer_active_{false};
  bool using_no_free_path_fallback_{false};
  bool no_free_path_wait_logged_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    free_paths_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    local_path_pub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_obstacle_cloud_;
  rclcpp::Time latest_obstacle_cloud_receive_time_;
  std::uint64_t latest_obstacle_cloud_version_{0};
  mutable std::mutex obstacle_cloud_mutex_;
  rclcpp::Time last_collision_warn_time_;
  rclcpp::Time last_free_paths_publish_time_;
};

}  // namespace pid_path_follower

#endif  // PID_PATH_FOLLOWER__PID_PATH_FOLLOWER_HPP_
