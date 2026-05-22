#include "pid_path_follower/pid_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pid_path_follower
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kPathNum = 343;
constexpr int kGroupNum = 7;
constexpr int kRotationNum = 36;
constexpr int kGridVoxelNumX = 161;
constexpr int kGridVoxelNumY = 451;
constexpr int kGridVoxelNum = kGridVoxelNumX * kGridVoxelNumY;

double distance2D(const geometry_msgs::msg::PoseStamped &a,
                  const geometry_msgs::msg::PoseStamped &b)
{
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
}
} // namespace

void PidPathFollower::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    if (!node_) {
        throw std::runtime_error("Failed to lock lifecycle node");
    }

    plugin_name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);

    declareParameters();
    if (use_path_library_) {
        loadPathLibrary();
    }
    resetPid();
    last_collision_warn_time_ = node_->now();

    if (collision_enabled_) {
        obstacle_cloud_sub_ =
            node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                collision_cloud_topic_, rclcpp::SensorDataQoS(),
                std::bind(&PidPathFollower::obstacleCloudCallback, this,
                          std::placeholders::_1));
    }
    if (visualize_free_paths_) {
        free_paths_pub_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                free_paths_topic_, rclcpp::QoS(1));
    }
    if (publish_local_costmap_path_) {
        local_path_pub_ =
            node_->create_publisher<nav_msgs::msg::Path>(
                local_costmap_path_topic_, rclcpp::QoS(1));
    }

    RCLCPP_INFO(node_->get_logger(),
                "Configured PID path follower '%s' with collision cloud '%s'",
                plugin_name_.c_str(), collision_cloud_topic_.c_str());
    RCLCPP_INFO(
        node_->get_logger(),
        "PID follower mode: use_local_costmap_planner=%s, use_path_library=%s, "
        "fallback_to_local_costmap_on_no_free_path=%s, fallback_delay=%.2fs",
        use_local_costmap_planner_ ? "true" : "false",
        use_path_library_ ? "true" : "false",
        fallback_to_local_costmap_on_no_free_path_ ? "true" : "false",
        no_free_path_fallback_delay_);
    if (use_local_costmap_planner_ &&
        fallback_to_local_costmap_on_no_free_path_) {
        RCLCPP_INFO(
            node_->get_logger(),
            "No-free-path fallback is inactive because local costmap planner is already enabled.");
    }
}

void PidPathFollower::cleanup()
{
    global_plan_.poses.clear();
    obstacle_cloud_sub_.reset();
    free_paths_pub_.reset();
    local_path_pub_.reset();
    {
        std::lock_guard<std::mutex> lock(obstacle_cloud_mutex_);
        latest_obstacle_cloud_.reset();
    }
    resetPid();
}

void PidPathFollower::activate()
{
    if (free_paths_pub_) {
        free_paths_pub_->on_activate();
    }
    if (local_path_pub_) {
        local_path_pub_->on_activate();
    }
    resetPid();
}

void PidPathFollower::deactivate()
{
    if (free_paths_pub_) {
        free_paths_pub_->on_deactivate();
    }
    if (local_path_pub_) {
        local_path_pub_->on_deactivate();
    }
    resetPid();
}

void PidPathFollower::setPlan(const nav_msgs::msg::Path &path)
{
    global_plan_ = path;
    linear_pid_ = PidState{};
    angular_pid_ = PidState{};
    previous_linear_cmd_ = 0.0;
    previous_angular_cmd_ = 0.0;
    has_previous_time_ = false;
    cached_selection_ = LocalPathSelection{};
    cached_collision_ = CollisionResult{};
    last_local_path_update_time_ =
        rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    has_cached_selection_ = false;
}

geometry_msgs::msg::TwistStamped PidPathFollower::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
    if (global_plan_.poses.empty()) {
        throw std::runtime_error("PID path follower received an empty path");
    }

    geometry_msgs::msg::PoseStamped final_pose;
    const auto global_target_pose = getLookaheadPose(pose, final_pose);

    if (goal_checker != nullptr &&
        goal_checker->isGoalReached(pose.pose, final_pose.pose, velocity)) {
        resetPid();
        return zeroCommand(pose);
    }

    const rclcpp::Time now = node_->now();
    CollisionResult collision;
    auto target_pose = global_target_pose;
    if (use_local_costmap_planner_) {
        if (shouldUpdateLocalPath(now)) {
            cached_selection_ = planLocalCostmapPath(pose, global_target_pose);
            cached_collision_ = CollisionResult{};
            if (!cached_selection_.found) {
                cached_collision_.blocked = true;
                cached_collision_.footprint_blocked = true;
                cached_collision_.nearest_distance = 0.0;
            }
            last_local_path_update_time_ = now;
            has_cached_selection_ = true;
        }

        collision = cached_collision_;
        if (cached_selection_.found) {
            target_pose = targetPoseFromLocalPath(pose, cached_selection_);
        } else {
            collision.blocked = true;
            collision.footprint_blocked = true;
            collision.nearest_distance = 0.0;
        }
    } else if (collision_enabled_ && use_path_library_ && path_library_loaded_) {
        if (shouldUpdateLocalPath(now)) {
            bool has_cloud = false;
            bool stale_cloud = false;
            auto obstacle_points =
                getObstaclePointsInRobotFrame(pose, has_cloud, stale_cloud);
            if (use_local_costmap_obstacles_) {
                auto costmap_points =
                    getLocalCostmapObstaclePointsInRobotFrame(pose);
                obstacle_points.insert(
                    obstacle_points.end(),
                    std::make_move_iterator(costmap_points.begin()),
                    std::make_move_iterator(costmap_points.end()));
            }
            collision.has_cloud = has_cloud;
            collision.stale_cloud = stale_cloud;

            const bool was_using_fallback = using_no_free_path_fallback_;
            const bool keep_fallback =
                using_no_free_path_fallback_ &&
                fallback_min_duration_ > 0.0 &&
                (now - fallback_start_time_).seconds() <
                    fallback_min_duration_;

            if (keep_fallback) {
                cached_selection_ = planLocalCostmapPath(pose, global_target_pose);
                cached_collision_ = CollisionResult{};
                if (cached_selection_.found) {
                    RCLCPP_INFO_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 1000,
                        "Holding local_costmap A* fallback for %.2f / %.2f s before trying free_path again.",
                        (now - fallback_start_time_).seconds(),
                        fallback_min_duration_);
                }
            } else {
                if (using_no_free_path_fallback_) {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "A* fallback minimum duration elapsed, trying free_path again.");
                    using_no_free_path_fallback_ = false;
                }

                const double active_max =
                    std::max(activeMaxLinearVelocity(), 0.001);
                const double speed_scale = std::clamp(
                    std::abs(previous_linear_cmd_) / active_max, 0.2, 1.0);
                cached_selection_ =
                    selectLibraryPath(pose, global_target_pose, final_pose,
                                      speed_scale, obstacle_points);
                cached_collision_ = collision;
            }

            if (cached_selection_.found && !using_no_free_path_fallback_) {
                if (was_using_fallback) {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "free_path recovered, exiting local_costmap A* fallback.");
                } else if (no_free_path_timer_active_) {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "free_path recovered before fallback delay elapsed.");
                }
                no_free_path_timer_active_ = false;
                using_no_free_path_fallback_ = false;
                no_free_path_wait_logged_ = false;
            } else if (!cached_selection_.found && was_using_fallback) {
                auto fallback_selection =
                    planLocalCostmapPath(pose, global_target_pose);
                if (fallback_selection.found) {
                    cached_selection_ = std::move(fallback_selection);
                    using_no_free_path_fallback_ = true;
                    fallback_start_time_ = now;
                    no_free_path_timer_active_ = false;
                    no_free_path_wait_logged_ = false;
                    cached_collision_ = CollisionResult{};
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "free_path is still unavailable after A* hold time; continuing local_costmap A* fallback for another %.2f s.",
                        fallback_min_duration_);
                } else {
                    RCLCPP_WARN_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 1000,
                        "free_path is still unavailable and local_costmap A* fallback failed.");
                }
            } else if (!cached_selection_.found) {
                if (!no_free_path_timer_active_) {
                    no_free_path_start_time_ = now;
                    no_free_path_timer_active_ = true;
                    no_free_path_wait_logged_ = false;
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "No free_path found; will try local_costmap A* fallback after %.2f s.",
                        no_free_path_fallback_delay_);
                }

                const double no_free_path_duration =
                    (now - no_free_path_start_time_).seconds();
                if (!no_free_path_wait_logged_ &&
                    no_free_path_duration >=
                        std::min(no_free_path_fallback_delay_, 1.0)) {
                    no_free_path_wait_logged_ = true;
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "No free_path for %.2f s, waiting for fallback threshold %.2f s.",
                        no_free_path_duration, no_free_path_fallback_delay_);
                }

                const bool fallback_ready =
                    fallback_to_local_costmap_on_no_free_path_ &&
                    no_free_path_fallback_delay_ >= 0.0 &&
                    no_free_path_duration >= no_free_path_fallback_delay_;
                if (fallback_ready) {
                    auto fallback_selection =
                        planLocalCostmapPath(pose, global_target_pose);
                    if (fallback_selection.found) {
                        cached_selection_ = std::move(fallback_selection);
                        if (!using_no_free_path_fallback_) {
                            RCLCPP_WARN(
                                node_->get_logger(),
                                "No free_path for %.2f s; switching to local_costmap A* fallback.",
                                no_free_path_duration);
                        }
                        using_no_free_path_fallback_ = true;
                        fallback_start_time_ = now;
                        no_free_path_timer_active_ = false;
                        no_free_path_wait_logged_ = false;
                        cached_collision_ = CollisionResult{};
                    } else {
                        RCLCPP_WARN_THROTTLE(
                            node_->get_logger(), *node_->get_clock(), 1000,
                            "No free_path for %.2f s; local_costmap A* fallback failed.",
                            no_free_path_duration);
                    }
                }
            }

            if (!cached_selection_.found) {
                cached_collision_.blocked = true;
                cached_collision_.footprint_blocked = true;
                cached_collision_.nearest_distance = 0.0;
                cached_collision_.point_count =
                    static_cast<int>(obstacle_points.size());
            }
            last_local_path_update_time_ = now;
            has_cached_selection_ = true;
        }

        collision = cached_collision_;
        if (cached_selection_.found) {
            target_pose = targetPoseFromLocalPath(pose, cached_selection_);
        } else {
            collision.blocked = true;
            collision.footprint_blocked = true;
            collision.nearest_distance = 0.0;
        }
    } else if (collision_enabled_) {
        bool has_cloud = false;
        bool stale_cloud = false;
        auto obstacle_points =
            getObstaclePointsInRobotFrame(pose, has_cloud, stale_cloud);
        if (use_local_costmap_obstacles_) {
            auto costmap_points =
                getLocalCostmapObstaclePointsInRobotFrame(pose);
            obstacle_points.insert(
                obstacle_points.end(),
                std::make_move_iterator(costmap_points.begin()),
                std::make_move_iterator(costmap_points.end()));
        }
        collision = evaluatePathCollision(pose, global_target_pose,
                                          obstacle_points);
        collision.has_cloud = has_cloud;
        collision.stale_cloud = stale_cloud;
    }

    double dt = 1.0 / 20.0;
    if (has_previous_time_) {
        dt = (now - previous_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            dt = 1.0 / 20.0;
        }
    }
    previous_time_ = now;
    has_previous_time_ = true;

    const double robot_yaw = yawFromQuaternion(pose.pose.orientation);
    const double dx = target_pose.pose.position.x - pose.pose.position.x;
    const double dy = target_pose.pose.position.y - pose.pose.position.y;
    const double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    const double heading_error = std::atan2(local_y, local_x);
    const double distance_error = std::hypot(local_x, local_y);
    const double distance_to_goal = distance2D(pose, final_pose);

    double angular_error = heading_error;
    if (distance_to_goal < rotate_to_goal_distance_) {
        angular_error = normalizeAngle(
            yawFromQuaternion(final_pose.pose.orientation) - robot_yaw);
    }

    double linear_error =
        distance_error * std::max(0.0, std::cos(heading_error));
    if (std::abs(heading_error) > rotate_to_heading_threshold_) {
        linear_error = 0.0;
    }

    double linear_cmd = computePid(linear_error, linear_kp_, linear_ki_,
                                   linear_kd_, linear_integral_limit_, dt,
                                   linear_pid_);
    double angular_cmd = computePid(angular_error, angular_kp_, angular_ki_,
                                    angular_kd_, angular_integral_limit_, dt,
                                    angular_pid_);

    const double slowdown_scale = std::clamp(
        distance_to_goal / std::max(goal_slowdown_radius_, 0.001), 0.0, 1.0);
    const double max_linear = activeMaxLinearVelocity() * slowdown_scale;
    linear_cmd = std::clamp(linear_cmd, 0.0, max_linear);
    if (linear_cmd > 0.0 && linear_cmd < min_linear_velocity_) {
        linear_cmd = std::min(min_linear_velocity_, max_linear);
    }

    angular_cmd =
        std::clamp(angular_cmd, -max_angular_velocity_, max_angular_velocity_);

    if (collision_enabled_ && collision.stale_cloud && stop_on_stale_cloud_) {
        resetPid();
        return zeroCommand(pose);
    }

    if (collision_enabled_ && collision.blocked) {
        warnCollisionThrottled(collision);
        if (collision.nearest_distance <= collision_stop_distance_) {
            linear_cmd = 0.0;
            if (collision.footprint_blocked) {
                angular_cmd = 0.0;
            }
        } else if (collision.nearest_distance < collision_slow_distance_) {
            const double scale = std::clamp(
                (collision.nearest_distance - collision_stop_distance_) /
                    std::max(collision_slow_distance_ -
                                 collision_stop_distance_,
                             0.001),
                collision_min_speed_scale_, 1.0);
            linear_cmd *= scale;
        }
    }

    linear_cmd =
        limitRate(linear_cmd, previous_linear_cmd_, max_linear_accel_, dt);
    angular_cmd =
        limitRate(angular_cmd, previous_angular_cmd_, max_angular_accel_, dt);

    if (collision_enabled_ && collision.blocked &&
        collision.nearest_distance <= collision_stop_distance_) {
        linear_cmd = 0.0;
        if (collision.footprint_blocked) {
            angular_cmd = 0.0;
        }
    }

    previous_linear_cmd_ = linear_cmd;
    previous_angular_cmd_ = angular_cmd;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = pose.header.frame_id;
    cmd.twist.linear.x = linear_cmd;
    cmd.twist.angular.z = angular_cmd;
    return cmd;
}

void PidPathFollower::setSpeedLimit(const double &speed_limit,
                                    const bool &percentage)
{
    if (speed_limit <= 0.0) {
        has_speed_limit_ = false;
        speed_limit_ = 0.0;
        return;
    }

    has_speed_limit_ = true;
    if (percentage) {
        speed_limit_ = max_linear_velocity_ * speed_limit / 100.0;
    } else {
        speed_limit_ = speed_limit;
    }
}

void PidPathFollower::declareParameters()
{
    auto declare_double = [this](const std::string &name,
                                 double default_value) {
        const std::string full_name = plugin_name_ + "." + name;
        if (!node_->has_parameter(full_name)) {
            node_->declare_parameter(full_name, default_value);
        }
        return node_->get_parameter(full_name).as_double();
    };
    auto declare_bool = [this](const std::string &name, bool default_value) {
        const std::string full_name = plugin_name_ + "." + name;
        if (!node_->has_parameter(full_name)) {
            node_->declare_parameter(full_name, default_value);
        }
        return node_->get_parameter(full_name).as_bool();
    };
    auto declare_int = [this](const std::string &name, int default_value) {
        const std::string full_name = plugin_name_ + "." + name;
        if (!node_->has_parameter(full_name)) {
            node_->declare_parameter(full_name, default_value);
        }
        return static_cast<int>(node_->get_parameter(full_name).as_int());
    };
    auto declare_string = [this](const std::string &name,
                                 const std::string &default_value) {
        const std::string full_name = plugin_name_ + "." + name;
        if (!node_->has_parameter(full_name)) {
            node_->declare_parameter(full_name, default_value);
        }
        return node_->get_parameter(full_name).as_string();
    };

    lookahead_distance_ =
        declare_double("lookahead_distance", lookahead_distance_);
    prune_distance_ = declare_double("prune_distance", prune_distance_);
    transform_tolerance_ =
        declare_double("transform_tolerance", transform_tolerance_);
    linear_kp_ = declare_double("linear_kp", linear_kp_);
    linear_ki_ = declare_double("linear_ki", linear_ki_);
    linear_kd_ = declare_double("linear_kd", linear_kd_);
    angular_kp_ = declare_double("angular_kp", angular_kp_);
    angular_ki_ = declare_double("angular_ki", angular_ki_);
    angular_kd_ = declare_double("angular_kd", angular_kd_);
    linear_integral_limit_ =
        declare_double("linear_integral_limit", linear_integral_limit_);
    angular_integral_limit_ =
        declare_double("angular_integral_limit", angular_integral_limit_);
    max_linear_velocity_ =
        declare_double("max_linear_velocity", max_linear_velocity_);
    min_linear_velocity_ =
        declare_double("min_linear_velocity", min_linear_velocity_);
    max_angular_velocity_ =
        declare_double("max_angular_velocity", max_angular_velocity_);
    max_linear_accel_ = declare_double("max_linear_accel", max_linear_accel_);
    max_angular_accel_ =
        declare_double("max_angular_accel", max_angular_accel_);
    goal_slowdown_radius_ =
        declare_double("goal_slowdown_radius", goal_slowdown_radius_);
    rotate_to_goal_distance_ =
        declare_double("rotate_to_goal_distance", rotate_to_goal_distance_);
    rotate_to_heading_threshold_ = declare_double("rotate_to_heading_threshold",
                                                  rotate_to_heading_threshold_);

    collision_enabled_ = declare_bool("collision_enabled", collision_enabled_);
    collision_cloud_topic_ =
        declare_string("collision_cloud_topic", collision_cloud_topic_);
    collision_cloud_timeout_ =
        declare_double("collision_cloud_timeout", collision_cloud_timeout_);
    stop_on_stale_cloud_ =
        declare_bool("stop_on_stale_cloud", stop_on_stale_cloud_);
    collision_voxel_size_ =
        declare_double("collision_voxel_size", collision_voxel_size_);
    collision_adjacent_range_ =
        declare_double("collision_adjacent_range", collision_adjacent_range_);
    collision_check_distance_ =
        declare_double("collision_check_distance", collision_check_distance_);
    collision_slow_distance_ =
        declare_double("collision_slow_distance", collision_slow_distance_);
    collision_stop_distance_ =
        declare_double("collision_stop_distance", collision_stop_distance_);
    collision_min_speed_scale_ =
        declare_double("collision_min_speed_scale", collision_min_speed_scale_);
    collision_lateral_margin_ =
        declare_double("collision_lateral_margin", collision_lateral_margin_);
    collision_min_z_ = declare_double("collision_min_z", collision_min_z_);
    collision_max_z_ = declare_double("collision_max_z", collision_max_z_);
    collision_rear_margin_ =
        declare_double("collision_rear_margin", collision_rear_margin_);
    vehicle_length_ = declare_double("vehicle_length", vehicle_length_);
    vehicle_width_ = declare_double("vehicle_width", vehicle_width_);
    collision_point_threshold_ =
        std::max(1, declare_int("collision_point_threshold",
                                collision_point_threshold_));

    use_path_library_ = declare_bool("use_path_library", use_path_library_);
    use_local_costmap_planner_ =
        declare_bool("use_local_costmap_planner", use_local_costmap_planner_);
    use_local_costmap_obstacles_ =
        declare_bool("use_local_costmap_obstacles", use_local_costmap_obstacles_);
    fallback_to_local_costmap_on_no_free_path_ =
        declare_bool("fallback_to_local_costmap_on_no_free_path",
                     fallback_to_local_costmap_on_no_free_path_);
    local_costmap_allow_unknown_ =
        declare_bool("local_costmap_allow_unknown", local_costmap_allow_unknown_);
    publish_local_costmap_path_ =
        declare_bool("publish_local_costmap_path", publish_local_costmap_path_);
    path_folder_ = declare_string("path_folder", path_folder_);
    two_way_drive_ = declare_bool("two_way_drive", two_way_drive_);
    check_rot_obstacle_ =
        declare_bool("check_rot_obstacle", check_rot_obstacle_);
    path_scale_by_speed_ =
        declare_bool("path_scale_by_speed", path_scale_by_speed_);
    path_range_by_speed_ =
        declare_bool("path_range_by_speed", path_range_by_speed_);
    path_crop_by_goal_ =
        declare_bool("path_crop_by_goal", path_crop_by_goal_);
    dir_to_vehicle_ = declare_bool("dir_to_vehicle", dir_to_vehicle_);
    path_scale_ = declare_double("path_scale", path_scale_);
    min_path_scale_ = declare_double("min_path_scale", min_path_scale_);
    path_scale_step_ = declare_double("path_scale_step", path_scale_step_);
    min_path_range_ = declare_double("min_path_range", min_path_range_);
    path_range_step_ = declare_double("path_range_step", path_range_step_);
    dir_weight_ = declare_double("dir_weight", dir_weight_);
    dir_threshold_deg_ =
        declare_double("dir_threshold_deg", dir_threshold_deg_);
    goal_clear_range_ = declare_double("goal_clear_range", goal_clear_range_);
    local_costmap_goal_distance_ =
        declare_double("local_costmap_goal_distance",
                       local_costmap_goal_distance_);
    local_costmap_cost_weight_ =
        declare_double("local_costmap_cost_weight", local_costmap_cost_weight_);
    no_free_path_fallback_delay_ =
        declare_double("no_free_path_fallback_delay",
                       no_free_path_fallback_delay_);
    fallback_min_duration_ =
        declare_double("fallback_min_duration", fallback_min_duration_);
    local_costmap_lethal_cost_ =
        std::clamp(declare_int("local_costmap_lethal_cost",
                               local_costmap_lethal_cost_), 1, 255);
    local_costmap_path_topic_ =
        declare_string("local_costmap_path_topic", local_costmap_path_topic_);
    visualize_free_paths_ =
        declare_bool("visualize_free_paths", visualize_free_paths_);
    free_paths_topic_ = declare_string("free_paths_topic", free_paths_topic_);
    free_paths_publish_period_ =
        declare_double("free_paths_publish_period", free_paths_publish_period_);
    local_path_update_period_ =
        declare_double("local_path_update_period", local_path_update_period_);
    candidate_rotation_step_ =
        std::max(1, declare_int("candidate_rotation_step",
                                candidate_rotation_step_));
    free_paths_rotation_step_ =
        std::max(1, declare_int("free_paths_rotation_step",
                                free_paths_rotation_step_));
    free_paths_point_skip_ =
        std::max(0, declare_int("free_paths_point_skip",
                                free_paths_point_skip_));
}

int PidPathFollower::readPlyVertexCount(std::istream &stream) const
{
    std::string token;
    int vertex_count = 0;
    while (stream >> token) {
        if (token == "element") {
            std::string element_name;
            stream >> element_name;
            if (element_name == "vertex") {
                stream >> vertex_count;
            }
        } else if (token == "end_header") {
            break;
        }
    }
    return vertex_count;
}

void PidPathFollower::loadPathLibrary()
{
    path_library_loaded_ = false;
    if (path_folder_.empty()) {
        try {
            path_folder_ =
                ament_index_cpp::get_package_share_directory(
                    "pid_path_follower") +
                "/paths";
        } catch (const std::exception &ex) {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot locate pid_path_follower share directory: %s",
                        ex.what());
            return;
        }
    }

    const std::string start_paths_file = path_folder_ + "/startPaths.ply";
    const std::string visual_paths_file = path_folder_ + "/paths.ply";
    const std::string path_list_file = path_folder_ + "/pathList.ply";
    const std::string correspondences_file =
        path_folder_ + "/correspondences.txt";

    start_paths_.assign(kGroupNum, {});
    visual_paths_.assign(kPathNum, {});
    path_group_ids_.assign(kPathNum, 0);
    path_end_direction_deg_.assign(kPathNum, 0.0);
    correspondences_.assign(kGridVoxelNum, {});

    std::ifstream start_paths_stream(start_paths_file);
    if (!start_paths_stream.is_open()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot open local planner start paths: %s",
                    start_paths_file.c_str());
        return;
    }
    const int start_path_points = readPlyVertexCount(start_paths_stream);
    for (int i = 0; i < start_path_points; ++i) {
        PathPoint point;
        int group_id = -1;
        if (!(start_paths_stream >> point.x >> point.y >> point.z >>
              group_id)) {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to read startPaths.ply at point %d", i);
            return;
        }
        if (group_id >= 0 && group_id < kGroupNum) {
            start_paths_[group_id].push_back(point);
        }
    }

    std::ifstream visual_paths_stream(visual_paths_file);
    if (!visual_paths_stream.is_open()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot open local planner visualization paths: %s",
                    visual_paths_file.c_str());
    } else {
        const int visual_path_points = readPlyVertexCount(visual_paths_stream);
        int visual_skip_count = 0;
        for (int i = 0; i < visual_path_points; ++i) {
            PathPoint point;
            int path_id = -1;
            double intensity = 0.0;
            if (!(visual_paths_stream >> point.x >> point.y >> point.z >>
                  path_id >> intensity)) {
                RCLCPP_WARN(node_->get_logger(),
                            "Failed to read paths.ply at point %d", i);
                visual_paths_.assign(kPathNum, {});
                break;
            }
            if (path_id >= 0 && path_id < kPathNum) {
                ++visual_skip_count;
                if (visual_skip_count > free_paths_point_skip_) {
                    visual_paths_[path_id].push_back(point);
                    visual_skip_count = 0;
                }
            }
        }
    }

    std::ifstream path_list_stream(path_list_file);
    if (!path_list_stream.is_open()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot open path list: %s",
                    path_list_file.c_str());
        return;
    }
    const int path_count = readPlyVertexCount(path_list_stream);
    if (path_count != kPathNum) {
        RCLCPP_WARN(node_->get_logger(),
                    "Path library has %d paths, expected %d", path_count,
                    kPathNum);
        return;
    }
    for (int i = 0; i < path_count; ++i) {
        double end_x = 0.0;
        double end_y = 0.0;
        double end_z = 0.0;
        int path_id = -1;
        int group_id = -1;
        if (!(path_list_stream >> end_x >> end_y >> end_z >> path_id >>
              group_id)) {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to read pathList.ply at path %d", i);
            return;
        }
        if (path_id >= 0 && path_id < kPathNum && group_id >= 0 &&
            group_id < kGroupNum) {
            path_group_ids_[path_id] = group_id;
            path_end_direction_deg_[path_id] =
                2.0 * std::atan2(end_y, end_x) * 180.0 / kPi;
        }
    }

    std::ifstream correspondences_stream(correspondences_file);
    if (!correspondences_stream.is_open()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot open correspondences: %s",
                    correspondences_file.c_str());
        return;
    }
    for (int i = 0; i < kGridVoxelNum; ++i) {
        int grid_voxel_id = -1;
        if (!(correspondences_stream >> grid_voxel_id)) {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to read correspondence row %d", i);
            return;
        }

        while (true) {
            int path_id = -1;
            if (!(correspondences_stream >> path_id)) {
                RCLCPP_WARN(node_->get_logger(),
                            "Unexpected end of correspondences at row %d", i);
                return;
            }
            if (path_id == -1) {
                break;
            }
            if (grid_voxel_id >= 0 && grid_voxel_id < kGridVoxelNum &&
                path_id >= 0 && path_id < kPathNum) {
                correspondences_[grid_voxel_id].push_back(path_id);
            }
        }
    }

    for (const auto &path : start_paths_) {
        if (path.empty()) {
            RCLCPP_WARN(node_->get_logger(),
                        "Path library contains an empty start path group");
            return;
        }
    }

    path_library_loaded_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "Loaded local planner path library from '%s'",
                path_folder_.c_str());
}

void PidPathFollower::resetPid()
{
    linear_pid_ = PidState{};
    angular_pid_ = PidState{};
    previous_linear_cmd_ = 0.0;
    previous_angular_cmd_ = 0.0;
    has_previous_time_ = false;
    cached_selection_ = LocalPathSelection{};
    cached_collision_ = CollisionResult{};
    last_local_path_update_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    has_cached_selection_ = false;
    no_free_path_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    fallback_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    no_free_path_timer_active_ = false;
    using_no_free_path_fallback_ = false;
    no_free_path_wait_logged_ = false;
}

double PidPathFollower::computePid(double error, double kp, double ki,
                                   double kd, double integral_limit, double dt,
                                   PidState &state)
{
    state.integral += error * dt;
    state.integral =
        std::clamp(state.integral, -integral_limit, integral_limit);

    double derivative = 0.0;
    if (state.has_previous_error && dt > 0.0) {
        derivative = (error - state.previous_error) / dt;
    }

    state.previous_error = error;
    state.has_previous_error = true;

    return kp * error + ki * state.integral + kd * derivative;
}

double PidPathFollower::limitRate(double desired, double previous, double limit,
                                  double dt) const
{
    if (limit <= 0.0 || dt <= 0.0) {
        return desired;
    }

    const double max_delta = limit * dt;
    return previous + std::clamp(desired - previous, -max_delta, max_delta);
}

double PidPathFollower::activeMaxLinearVelocity() const
{
    if (!has_speed_limit_) {
        return max_linear_velocity_;
    }

    return std::clamp(speed_limit_, 0.0, max_linear_velocity_);
}

double PidPathFollower::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion &orientation) const
{
    return tf2::getYaw(orientation);
}

double PidPathFollower::normalizeAngle(double angle) const
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::PoseStamped
PidPathFollower::transformPose(const geometry_msgs::msg::PoseStamped &pose,
                               const std::string &target_frame) const
{
    if (pose.header.frame_id == target_frame) {
        return pose;
    }

    geometry_msgs::msg::PoseStamped transformed_pose;
    tf_->transform(pose, transformed_pose, target_frame,
                   tf2::durationFromSec(transform_tolerance_));
    return transformed_pose;
}

geometry_msgs::msg::PoseStamped PidPathFollower::getLookaheadPose(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    geometry_msgs::msg::PoseStamped &final_pose) const
{
    const std::string &target_frame = robot_pose.header.frame_id;
    final_pose = transformPose(global_plan_.poses.back(), target_frame);

    double best_distance = std::numeric_limits<double>::max();
    std::size_t closest_index = 0;
    bool found_pose = false;

    for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
        try {
            const auto candidate =
                transformPose(global_plan_.poses[i], target_frame);
            const double distance = distance2D(robot_pose, candidate);
            if (distance < best_distance) {
                best_distance = distance;
                closest_index = i;
                found_pose = true;
            }
        } catch (const tf2::TransformException &) {
            continue;
        }
    }

    if (!found_pose) {
        throw std::runtime_error(
            "Failed to transform any path pose into controller frame");
    }

    for (std::size_t i = closest_index; i < global_plan_.poses.size(); ++i) {
        const auto candidate =
            transformPose(global_plan_.poses[i], target_frame);
        const double distance = distance2D(robot_pose, candidate);
        if (distance >= lookahead_distance_ && distance >= prune_distance_) {
            return candidate;
        }
    }

    return final_pose;
}

geometry_msgs::msg::TwistStamped
PidPathFollower::zeroCommand(const geometry_msgs::msg::PoseStamped &pose) const
{
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_->now();
    cmd.header.frame_id = pose.header.frame_id;
    return cmd;
}

void PidPathFollower::obstacleCloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
    std::lock_guard<std::mutex> lock(obstacle_cloud_mutex_);
    latest_obstacle_cloud_ = std::move(cloud);
    latest_obstacle_cloud_receive_time_ = node_->now();
    ++latest_obstacle_cloud_version_;
}

std::vector<PidPathFollower::LocalPoint>
PidPathFollower::getObstaclePointsInRobotFrame(
    const geometry_msgs::msg::PoseStamped &robot_pose, bool &has_cloud,
    bool &stale_cloud) const
{
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    rclcpp::Time receive_time;
    {
        std::lock_guard<std::mutex> lock(obstacle_cloud_mutex_);
        cloud = latest_obstacle_cloud_;
        receive_time = latest_obstacle_cloud_receive_time_;
    }

    has_cloud = static_cast<bool>(cloud);
    stale_cloud = false;
    std::vector<LocalPoint> points;
    if (!cloud) {
        return points;
    }

    if (collision_cloud_timeout_ > 0.0) {
        const double age = (node_->now() - receive_time).seconds();
        stale_cloud = age > collision_cloud_timeout_;
    }

    const std::string cloud_frame = cloud->header.frame_id.empty() ?
                                        robot_pose.header.frame_id :
                                        cloud->header.frame_id;
    tf2::Transform cloud_to_target;
    cloud_to_target.setIdentity();
    if (cloud_frame != robot_pose.header.frame_id) {
        try {
            const auto transform = tf_->lookupTransform(
                robot_pose.header.frame_id, cloud_frame, tf2::TimePointZero,
                tf2::durationFromSec(transform_tolerance_));
            tf2::fromMsg(transform.transform, cloud_to_target);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "Failed to transform collision cloud from '%s' to '%s': %s",
                cloud_frame.c_str(), robot_pose.header.frame_id.c_str(),
                ex.what());
            return points;
        }
    }

    const double robot_yaw = yawFromQuaternion(robot_pose.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    std::unordered_set<std::int64_t> voxel_keys;
    if (collision_voxel_size_ > 0.0) {
        voxel_keys.reserve(cloud->width * cloud->height);
    }
    points.reserve(std::min<std::size_t>(cloud->width * cloud->height, 4096));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) ||
            !std::isfinite(*iter_z)) {
            continue;
        }

        tf2::Vector3 point_in_target(*iter_x, *iter_y, *iter_z);
        point_in_target = cloud_to_target * point_in_target;

        const double dx = point_in_target.x() - robot_pose.pose.position.x;
        const double dy = point_in_target.y() - robot_pose.pose.position.y;
        const double local_x = cos_yaw * dx + sin_yaw * dy;
        const double local_y = -sin_yaw * dx + cos_yaw * dy;
        const double local_z = point_in_target.z() - robot_pose.pose.position.z;

        if (local_z < collision_min_z_ || local_z > collision_max_z_) {
            continue;
        }
        if (std::hypot(local_x, local_y) > collision_adjacent_range_) {
            continue;
        }

        if (collision_voxel_size_ > 0.0) {
            const auto ix = static_cast<std::int64_t>(
                std::floor(local_x / collision_voxel_size_));
            const auto iy = static_cast<std::int64_t>(
                std::floor(local_y / collision_voxel_size_));
            const auto iz = static_cast<std::int64_t>(
                std::floor(local_z / collision_voxel_size_));
            const std::int64_t key = ix * 73856093LL ^ iy * 19349663LL ^
                                     iz * 83492791LL;
            if (!voxel_keys.insert(key).second) {
                continue;
            }
        }

        points.push_back({ local_x, local_y, local_z });
    }

    return points;
}

std::vector<PidPathFollower::LocalPoint>
PidPathFollower::getLocalCostmapObstaclePointsInRobotFrame(
    const geometry_msgs::msg::PoseStamped &robot_pose) const
{
    std::vector<LocalPoint> points;
    if (!costmap_ros_ || costmap_ros_->getCostmap() == nullptr) {
        return points;
    }

    auto *costmap = costmap_ros_->getCostmap();
    geometry_msgs::msg::PoseStamped robot_in_costmap;
    try {
        robot_in_costmap =
            transformPose(robot_pose, costmap_ros_->getGlobalFrameID());
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Failed to transform robot pose into local costmap frame: %s",
            ex.what());
        return points;
    }

    const double robot_yaw = yawFromQuaternion(robot_in_costmap.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();
    points.reserve(size_x * size_y / 8);

    for (unsigned int mx = 0; mx < size_x; ++mx) {
        for (unsigned int my = 0; my < size_y; ++my) {
            if (!isCostmapCellBlocked(mx, my)) {
                continue;
            }

            double wx = 0.0;
            double wy = 0.0;
            costmap->mapToWorld(mx, my, wx, wy);
            const double dx = wx - robot_in_costmap.pose.position.x;
            const double dy = wy - robot_in_costmap.pose.position.y;
            const double local_x = cos_yaw * dx + sin_yaw * dy;
            const double local_y = -sin_yaw * dx + cos_yaw * dy;
            if (std::hypot(local_x, local_y) > collision_adjacent_range_) {
                continue;
            }

            points.push_back({local_x, local_y, 0.0});
        }
    }

    return points;
}

std::vector<std::pair<double, double>> PidPathFollower::buildLocalPath(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PoseStamped &target_pose) const
{
    std::vector<std::pair<double, double>> local_path;
    local_path.emplace_back(0.0, 0.0);

    const double robot_yaw = yawFromQuaternion(robot_pose.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    auto to_local = [&](const geometry_msgs::msg::PoseStamped &pose) {
        const double dx = pose.pose.position.x - robot_pose.pose.position.x;
        const double dy = pose.pose.position.y - robot_pose.pose.position.y;
        return std::pair<double, double>{ cos_yaw * dx + sin_yaw * dy,
                                          -sin_yaw * dx + cos_yaw * dy };
    };

    double best_distance = std::numeric_limits<double>::max();
    std::size_t closest_index = 0;
    bool found_pose = false;
    for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
        try {
            const auto candidate = transformPose(global_plan_.poses[i],
                                                 robot_pose.header.frame_id);
            const double distance = distance2D(robot_pose, candidate);
            if (distance < best_distance) {
                best_distance = distance;
                closest_index = i;
                found_pose = true;
            }
        } catch (const tf2::TransformException &) {
            continue;
        }
    }

    double accumulated = 0.0;
    if (found_pose) {
        for (std::size_t i = closest_index; i < global_plan_.poses.size();
             ++i) {
            try {
                const auto candidate = transformPose(
                    global_plan_.poses[i], robot_pose.header.frame_id);
                const auto local = to_local(candidate);
                const double step =
                    std::hypot(local.first - local_path.back().first,
                               local.second - local_path.back().second);
                if (step < 0.03) {
                    continue;
                }

                accumulated += step;
                local_path.push_back(local);
                if (accumulated >= collision_check_distance_) {
                    break;
                }
            } catch (const tf2::TransformException &) {
                continue;
            }
        }
    }

    if (local_path.size() == 1) {
        local_path.push_back(to_local(target_pose));
    }

    return local_path;
}

PidPathFollower::CollisionResult PidPathFollower::evaluatePathCollision(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PoseStamped &target_pose,
    const std::vector<LocalPoint> &obstacle_points) const
{
    CollisionResult result;
    if (obstacle_points.empty()) {
        return result;
    }

    const auto local_path = buildLocalPath(robot_pose, target_pose);
    if (local_path.size() < 2) {
        return result;
    }

    const double half_width =
        std::max(0.01, vehicle_width_ * 0.5 + collision_lateral_margin_);
    const double half_length = std::max(0.01, vehicle_length_ * 0.5);
    const double check_distance =
        std::max(collision_check_distance_, lookahead_distance_);
    double nearest_distance = std::numeric_limits<double>::max();
    int hit_count = 0;
    int footprint_hit_count = 0;

    std::vector<double> cumulative(local_path.size(), 0.0);
    for (std::size_t i = 1; i < local_path.size(); ++i) {
        cumulative[i] =
            cumulative[i - 1] +
            std::hypot(local_path[i].first - local_path[i - 1].first,
                       local_path[i].second - local_path[i - 1].second);
    }

    for (const auto &point : obstacle_points) {
        if (point.x < -collision_rear_margin_) {
            continue;
        }

        if (point.x >= -half_length && point.x <= half_length &&
            std::abs(point.y) <= half_width) {
            ++footprint_hit_count;
            nearest_distance = 0.0;
            continue;
        }

        for (std::size_t i = 1; i < local_path.size(); ++i) {
            const double segment_length = cumulative[i] - cumulative[i - 1];
            if (segment_length <= 1e-6 || cumulative[i - 1] > check_distance) {
                continue;
            }

            double fraction = 0.0;
            const double distance =
                distanceToSegment(point.x, point.y, local_path[i - 1].first,
                                  local_path[i - 1].second, local_path[i].first,
                                  local_path[i].second, fraction);
            const double along =
                cumulative[i - 1] +
                std::clamp(fraction, 0.0, 1.0) * segment_length;
            if (along > check_distance) {
                continue;
            }

            if (distance <= half_width) {
                ++hit_count;
                nearest_distance = std::min(nearest_distance, along);
                break;
            }
        }
    }

    result.point_count = hit_count + footprint_hit_count;
    result.footprint_blocked = footprint_hit_count >=
                               collision_point_threshold_;
    result.blocked = result.footprint_blocked ||
                     hit_count >= collision_point_threshold_;
    if (result.blocked) {
        result.nearest_distance =
            std::isfinite(nearest_distance) ? nearest_distance : 0.0;
    }

    return result;
}

PidPathFollower::LocalPathSelection PidPathFollower::selectLibraryPath(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PoseStamped &global_target_pose,
    const geometry_msgs::msg::PoseStamped &final_pose, double speed_scale,
    const std::vector<LocalPoint> &obstacle_points)
{
    LocalPathSelection selection;
    if (!path_library_loaded_) {
        return selection;
    }

    const double robot_yaw = yawFromQuaternion(robot_pose.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    auto to_local = [&](const geometry_msgs::msg::PoseStamped &pose) {
        const double dx = pose.pose.position.x - robot_pose.pose.position.x;
        const double dy = pose.pose.position.y - robot_pose.pose.position.y;
        return std::pair<double, double>{
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy};
    };

    const auto target_local = to_local(global_target_pose);
    double desired_direction_deg =
        std::atan2(target_local.second, target_local.first) * 180.0 / kPi;
    if (!two_way_drive_) {
        desired_direction_deg =
            std::clamp(desired_direction_deg, -90.0, 90.0);
    }

    const auto final_local = to_local(final_pose);
    const double relative_goal_dis =
        std::max(0.05, std::hypot(final_local.first, final_local.second));

    double path_range = collision_adjacent_range_;
    if (path_range_by_speed_) {
        path_range = collision_adjacent_range_ * speed_scale;
    }
    path_range = std::max(path_range, min_path_range_);

    const double default_path_scale = path_scale_;
    double current_path_scale = default_path_scale;
    if (path_scale_by_speed_) {
        current_path_scale = default_path_scale * speed_scale;
    }
    current_path_scale = std::max(current_path_scale, min_path_scale_);

    while (current_path_scale >= min_path_scale_ &&
           path_range >= min_path_range_) {
        std::vector<int> clear_path_list(kRotationNum * kPathNum, 0);
        std::vector<double> clear_group_score(kRotationNum * kGroupNum, 0.0);

        double min_obs_ang_cw = -180.0;
        double min_obs_ang_ccw = 180.0;
        const double diameter =
            std::hypot(vehicle_length_ * 0.5, vehicle_width_ * 0.5);
        const double ang_offset =
            std::atan2(vehicle_width_, vehicle_length_) * 180.0 / kPi;

        for (const auto &point : obstacle_points) {
            const double x = point.x / current_path_scale;
            const double y = point.y / current_path_scale;
            const double dis = std::hypot(x, y);
            if (dis < path_range / current_path_scale &&
                (dis <= (relative_goal_dis + goal_clear_range_) /
                            current_path_scale ||
                 !path_crop_by_goal_)) {
                for (int rot_dir = 0; rot_dir < kRotationNum;
                     rot_dir += candidate_rotation_step_) {
                    if (!directionAllowed(desired_direction_deg, rot_dir)) {
                        continue;
                    }

                    const double rot_ang =
                        (10.0 * rot_dir - 180.0) * kPi / 180.0;
                    const double x2 =
                        std::cos(rot_ang) * x + std::sin(rot_ang) * y;
                    const double y2 =
                        -std::sin(rot_ang) * x + std::cos(rot_ang) * y;
                    const double scale_y =
                        x2 / grid_voxel_offset_x_ +
                        search_radius_ / grid_voxel_offset_y_ *
                            (grid_voxel_offset_x_ - x2) /
                            grid_voxel_offset_x_;
                    if (scale_y <= 1e-3) {
                        continue;
                    }

                    const int ind_x = static_cast<int>(
                        (grid_voxel_offset_x_ + grid_voxel_size_ * 0.5 - x2) /
                        grid_voxel_size_);
                    const int ind_y = static_cast<int>(
                        (grid_voxel_offset_y_ + grid_voxel_size_ * 0.5 -
                         y2 / scale_y) /
                        grid_voxel_size_);
                    if (ind_x >= 0 && ind_x < kGridVoxelNumX &&
                        ind_y >= 0 && ind_y < kGridVoxelNumY) {
                        const int ind = kGridVoxelNumY * ind_x + ind_y;
                        for (const int path_id : correspondences_[ind]) {
                            ++clear_path_list[kPathNum * rot_dir + path_id];
                        }
                    }
                }
            }

            if (check_rot_obstacle_ && dis < diameter / current_path_scale &&
                (std::abs(x) > vehicle_length_ / current_path_scale * 0.5 ||
                 std::abs(y) > vehicle_width_ / current_path_scale * 0.5)) {
                const double ang_obs = std::atan2(y, x) * 180.0 / kPi;
                if (ang_obs > 0.0) {
                    min_obs_ang_ccw =
                        std::min(min_obs_ang_ccw, ang_obs - ang_offset);
                    min_obs_ang_cw = std::max(
                        min_obs_ang_cw, ang_obs + ang_offset - 180.0);
                } else {
                    min_obs_ang_cw =
                        std::max(min_obs_ang_cw, ang_obs + ang_offset);
                    min_obs_ang_ccw = std::min(
                        min_obs_ang_ccw, 180.0 + ang_obs - ang_offset);
                }
            }
        }

        if (min_obs_ang_cw > 0.0) {
            min_obs_ang_cw = 0.0;
        }
        if (min_obs_ang_ccw < 0.0) {
            min_obs_ang_ccw = 0.0;
        }

        for (int rot_dir = 0; rot_dir < kRotationNum;
             rot_dir += candidate_rotation_step_) {
            for (int path_id = 0; path_id < kPathNum; ++path_id) {
                const int i = kPathNum * rot_dir + path_id;
            if (!directionAllowed(desired_direction_deg, rot_dir)) {
                continue;
            }

            if (clear_path_list[i] < collision_point_threshold_) {
                double dir_diff =
                    std::abs(desired_direction_deg -
                             path_end_direction_deg_[path_id] -
                             (10.0 * rot_dir - 180.0));
                if (dir_diff > 360.0) {
                    dir_diff -= 360.0;
                }
                if (dir_diff > 180.0) {
                    dir_diff = 360.0 - dir_diff;
                }

                double rot_dir_weight = 0.0;
                if (rot_dir < 18) {
                    rot_dir_weight = std::abs(std::abs(rot_dir - 9) + 1);
                } else {
                    rot_dir_weight = std::abs(std::abs(rot_dir - 27) + 1);
                }

                const double score =
                    (1.0 - std::sqrt(std::sqrt(dir_weight_ * dir_diff))) *
                    rot_dir_weight * rot_dir_weight * rot_dir_weight *
                    rot_dir_weight;
                if (score > 0.0) {
                    clear_group_score[kGroupNum * rot_dir +
                                      path_group_ids_[path_id]] += score;
                }
            }
            }
        }

        double max_score = 0.0;
        int selected_index = -1;
        for (int rot_dir = 0; rot_dir < kRotationNum;
             rot_dir += candidate_rotation_step_) {
            for (int group_id = 0; group_id < kGroupNum; ++group_id) {
            const int i = kGroupNum * rot_dir + group_id;
            const double rot_ang = (10.0 * rot_dir - 180.0) * kPi / 180.0;
            double rot_deg = 10.0 * rot_dir;
            if (rot_deg > 180.0) {
                rot_deg -= 360.0;
            }
            const bool rotation_clear =
                (rot_ang * 180.0 / kPi > min_obs_ang_cw &&
                 rot_ang * 180.0 / kPi < min_obs_ang_ccw) ||
                (rot_deg > min_obs_ang_cw && rot_deg < min_obs_ang_ccw &&
                 two_way_drive_) ||
                !check_rot_obstacle_;
            if (clear_group_score[i] > max_score && rotation_clear) {
                max_score = clear_group_score[i];
                selected_index = i;
            }
            }
        }

        if (selected_index >= 0) {
            const int rot_dir = selected_index / kGroupNum;
            const int group_id = selected_index % kGroupNum;
            const double rot_ang = (10.0 * rot_dir - 180.0) * kPi / 180.0;

            selection.local_path.clear();
            for (const auto &point : start_paths_[group_id]) {
                const double dis = std::hypot(point.x, point.y);
                if (dis <= path_range / current_path_scale &&
                    dis <= relative_goal_dis / current_path_scale) {
                    selection.local_path.emplace_back(
                        current_path_scale *
                            (std::cos(rot_ang) * point.x -
                             std::sin(rot_ang) * point.y),
                        current_path_scale *
                            (std::sin(rot_ang) * point.x +
                             std::cos(rot_ang) * point.y));
                } else {
                    break;
                }
            }

            if (selection.local_path.size() >= 2) {
                publishFreePaths(robot_pose, desired_direction_deg,
                                 current_path_scale, path_range,
                                 relative_goal_dis, clear_path_list,
                                 min_obs_ang_cw, min_obs_ang_ccw);
                selection.found = true;
                selection.path_scale = current_path_scale;
                selection.rotation_index = rot_dir;
                selection.group_index = group_id;
                return selection;
            }
        }

        if (current_path_scale >= min_path_scale_ + path_scale_step_) {
            current_path_scale -= path_scale_step_;
            path_range = collision_adjacent_range_ * current_path_scale /
                         default_path_scale;
            path_range = std::max(path_range, min_path_range_);
        } else {
            path_range -= path_range_step_;
        }
    }

    selection.blocked = true;
    publishEmptyFreePaths(robot_pose);
    return selection;
}

PidPathFollower::LocalPathSelection PidPathFollower::planLocalCostmapPath(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PoseStamped &fallback_goal)
{
    LocalPathSelection selection;
    if (!costmap_ros_) {
        return selection;
    }

    auto *costmap = costmap_ros_->getCostmap();
    if (costmap == nullptr) {
        return selection;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped goal_pose;
    try {
        start_pose = transformPose(robot_pose, costmap_ros_->getGlobalFrameID());
        goal_pose = getLocalCostmapGoal(start_pose, fallback_goal);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Failed to prepare local costmap plan: %s", ex.what());
        return selection;
    }

    unsigned int start_mx = 0;
    unsigned int start_my = 0;
    unsigned int goal_mx = 0;
    unsigned int goal_my = 0;
    if (!costmap->worldToMap(
            start_pose.pose.position.x, start_pose.pose.position.y,
            start_mx, start_my) ||
        !costmap->worldToMap(
            goal_pose.pose.position.x, goal_pose.pose.position.y,
            goal_mx, goal_my)) {
        return selection;
    }

    if (isCostmapCellBlocked(goal_mx, goal_my)) {
        return selection;
    }

    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();
    const auto to_index = [size_x](unsigned int mx, unsigned int my) {
        return static_cast<int>(my * size_x + mx);
    };
    const auto in_bounds = [size_x, size_y](int mx, int my) {
        return mx >= 0 && my >= 0 &&
               mx < static_cast<int>(size_x) &&
               my < static_cast<int>(size_y);
    };
    const auto heuristic = [](int ax, int ay, int bx, int by) {
        return std::hypot(static_cast<double>(ax - bx),
                          static_cast<double>(ay - by));
    };

    struct QueueNode
    {
        int index{0};
        double score{0.0};
        bool operator<(const QueueNode &other) const
        {
            return score > other.score;
        }
    };

    const int cell_count = static_cast<int>(size_x * size_y);
    std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cell_count, -1);
    std::priority_queue<QueueNode> open;

    const int start_index = to_index(start_mx, start_my);
    const int goal_index = to_index(goal_mx, goal_my);
    g_score[start_index] = 0.0;
    open.push({start_index, heuristic(
        static_cast<int>(start_mx), static_cast<int>(start_my),
        static_cast<int>(goal_mx), static_cast<int>(goal_my))});

    constexpr int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    constexpr int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    int reached_index = -1;

    while (!open.empty()) {
        const auto current = open.top();
        open.pop();
        if (current.index == goal_index) {
            reached_index = current.index;
            break;
        }

        const int cx = current.index % static_cast<int>(size_x);
        const int cy = current.index / static_cast<int>(size_x);
        for (int i = 0; i < 8; ++i) {
            const int nx = cx + dx[i];
            const int ny = cy + dy[i];
            if (!in_bounds(nx, ny)) {
                continue;
            }

            const auto nmx = static_cast<unsigned int>(nx);
            const auto nmy = static_cast<unsigned int>(ny);
            if (isCostmapCellBlocked(nmx, nmy)) {
                continue;
            }

            const int neighbor_index = to_index(nmx, nmy);
            const unsigned char cost = costmap->getCost(nmx, nmy);
            const double step = (dx[i] == 0 || dy[i] == 0) ? 1.0 : 1.41421356237;
            const double cost_scale =
                1.0 + local_costmap_cost_weight_ *
                static_cast<double>(cost) /
                std::max(1.0, static_cast<double>(local_costmap_lethal_cost_));
            const double tentative = g_score[current.index] + step * cost_scale;
            if (tentative < g_score[neighbor_index]) {
                g_score[neighbor_index] = tentative;
                parent[neighbor_index] = current.index;
                open.push({neighbor_index, tentative + heuristic(
                    nx, ny,
                    static_cast<int>(goal_mx), static_cast<int>(goal_my))});
            }
        }
    }

    if (reached_index < 0) {
        return selection;
    }

    std::vector<int> path_indices;
    for (int index = reached_index; index >= 0; index = parent[index]) {
        path_indices.push_back(index);
        if (index == start_index) {
            break;
        }
    }
    if (path_indices.empty() || path_indices.back() != start_index) {
        return selection;
    }
    std::reverse(path_indices.begin(), path_indices.end());

    const double robot_yaw = yawFromQuaternion(start_pose.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    selection.local_path.clear();
    selection.local_path.reserve(path_indices.size());
    for (const int index : path_indices) {
        const unsigned int mx = static_cast<unsigned int>(
            index % static_cast<int>(size_x));
        const unsigned int my = static_cast<unsigned int>(
            index / static_cast<int>(size_x));
        double wx = 0.0;
        double wy = 0.0;
        costmap->mapToWorld(mx, my, wx, wy);
        const double rel_x = wx - start_pose.pose.position.x;
        const double rel_y = wy - start_pose.pose.position.y;
        selection.local_path.emplace_back(
            cos_yaw * rel_x + sin_yaw * rel_y,
            -sin_yaw * rel_x + cos_yaw * rel_y);
    }

    if (selection.local_path.size() < 2) {
        return selection;
    }

    selection.found = true;
    publishLocalCostmapPath(start_pose, selection);
    return selection;
}

geometry_msgs::msg::PoseStamped PidPathFollower::getLocalCostmapGoal(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PoseStamped &fallback_goal) const
{
    if (!costmap_ros_ || costmap_ros_->getCostmap() == nullptr) {
        return fallback_goal;
    }

    auto *costmap = costmap_ros_->getCostmap();
    const std::string target_frame = costmap_ros_->getGlobalFrameID();
    geometry_msgs::msg::PoseStamped best_goal = transformPose(fallback_goal, target_frame);
    double best_distance = 0.0;
    double closest_distance = std::numeric_limits<double>::max();
    std::size_t closest_index = 0;
    bool found_closest = false;

    for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
        try {
            const auto candidate = transformPose(global_plan_.poses[i], target_frame);
            const double distance = distance2D(robot_pose, candidate);
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
                found_closest = true;
            }
        } catch (const tf2::TransformException &) {
            continue;
        }
    }

    if (!found_closest) {
        return best_goal;
    }

    for (std::size_t i = closest_index; i < global_plan_.poses.size(); ++i) {
        try {
            const auto candidate = transformPose(global_plan_.poses[i], target_frame);
            unsigned int mx = 0;
            unsigned int my = 0;
            if (!costmap->worldToMap(
                    candidate.pose.position.x, candidate.pose.position.y,
                    mx, my)) {
                continue;
            }
            if (isCostmapCellBlocked(mx, my)) {
                continue;
            }

            const double distance = distance2D(robot_pose, candidate);
            if (distance > best_distance) {
                best_distance = distance;
                best_goal = candidate;
            }
            if (distance >= local_costmap_goal_distance_) {
                break;
            }
        } catch (const tf2::TransformException &) {
            continue;
        }
    }

    return best_goal;
}

bool PidPathFollower::isCostmapCellBlocked(unsigned int mx, unsigned int my) const
{
    if (!costmap_ros_ || costmap_ros_->getCostmap() == nullptr) {
        return true;
    }

    const unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
    if (cost == 255) {
        return !local_costmap_allow_unknown_;
    }
    return cost >= static_cast<unsigned char>(local_costmap_lethal_cost_);
}

bool PidPathFollower::directionAllowed(double desired_direction_deg,
                                       int rotation_index) const
{
    const double rot_deg = 10.0 * rotation_index - 180.0;
    double angle_diff = std::abs(desired_direction_deg - rot_deg);
    if (angle_diff > 180.0) {
        angle_diff = 360.0 - angle_diff;
    }

    if ((angle_diff > dir_threshold_deg_ && !dir_to_vehicle_) ||
        (std::abs(rot_deg) > dir_threshold_deg_ &&
         std::abs(desired_direction_deg) <= 90.0 && dir_to_vehicle_) ||
        ((10.0 * rotation_index > dir_threshold_deg_ &&
          360.0 - 10.0 * rotation_index > dir_threshold_deg_) &&
         std::abs(desired_direction_deg) > 90.0 && dir_to_vehicle_)) {
        return false;
    }
    return true;
}

bool PidPathFollower::shouldPublishFreePaths()
{
    if (!visualize_free_paths_ || !free_paths_pub_) {
        return false;
    }
    if (free_paths_pub_->get_subscription_count() == 0) {
        return false;
    }

    const rclcpp::Time now = node_->now();
    if (last_free_paths_publish_time_.nanoseconds() != 0 &&
        free_paths_publish_period_ > 0.0 &&
        (now - last_free_paths_publish_time_).seconds() <
            free_paths_publish_period_) {
        return false;
    }
    last_free_paths_publish_time_ = now;
    return true;
}

bool PidPathFollower::shouldUpdateLocalPath(const rclcpp::Time &now) const
{
    if (!has_cached_selection_) {
        return true;
    }
    if (local_path_update_period_ <= 0.0) {
        return true;
    }
    return (now - last_local_path_update_time_).seconds() >=
           local_path_update_period_;
}

void PidPathFollower::publishFreePaths(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    double desired_direction_deg, double path_scale, double path_range,
    double relative_goal_distance, const std::vector<int> &clear_path_list,
    double min_obs_ang_cw, double min_obs_ang_ccw)
{
    if (!shouldPublishFreePaths()) {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    if (robot_pose.header.stamp.nanosec == 0 &&
        robot_pose.header.stamp.sec == 0) {
        cloud.header.stamp = node_->now();
    } else {
        cloud.header.stamp = robot_pose.header.stamp;
    }
    cloud.header.frame_id =
        costmap_ros_ ? costmap_ros_->getBaseFrameID() : "base_link";

    std::size_t point_count = 0;
    for (int rot_dir = 0; rot_dir < kRotationNum;
         rot_dir += free_paths_rotation_step_) {
        for (int path_id = 0; path_id < kPathNum; ++path_id) {
        const int i = kPathNum * rot_dir + path_id;
        if (clear_path_list[i] >= collision_point_threshold_ ||
            !directionAllowed(desired_direction_deg, rot_dir)) {
            continue;
        }

        const double rot_ang = (10.0 * rot_dir - 180.0) * kPi / 180.0;
        double rot_deg = 10.0 * rot_dir;
        if (rot_deg > 180.0) {
            rot_deg -= 360.0;
        }
        const bool rotation_clear =
            (rot_ang * 180.0 / kPi > min_obs_ang_cw &&
             rot_ang * 180.0 / kPi < min_obs_ang_ccw) ||
            (rot_deg > min_obs_ang_cw && rot_deg < min_obs_ang_ccw &&
             two_way_drive_) ||
            !check_rot_obstacle_;
        if (!rotation_clear) {
            continue;
        }

        point_count += visual_paths_[path_id].size();
        }
    }

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    std::size_t used_points = 0;

    for (int rot_dir = 0; rot_dir < kRotationNum;
         rot_dir += free_paths_rotation_step_) {
        for (int path_id = 0; path_id < kPathNum; ++path_id) {
        const int i = kPathNum * rot_dir + path_id;
        if (clear_path_list[i] >= collision_point_threshold_ ||
            !directionAllowed(desired_direction_deg, rot_dir)) {
            continue;
        }

        const double rot_ang = (10.0 * rot_dir - 180.0) * kPi / 180.0;
        double rot_deg = 10.0 * rot_dir;
        if (rot_deg > 180.0) {
            rot_deg -= 360.0;
        }
        const bool rotation_clear =
            (rot_ang * 180.0 / kPi > min_obs_ang_cw &&
             rot_ang * 180.0 / kPi < min_obs_ang_ccw) ||
            (rot_deg > min_obs_ang_cw && rot_deg < min_obs_ang_ccw &&
             two_way_drive_) ||
            !check_rot_obstacle_;
        if (!rotation_clear) {
            continue;
        }

        for (const auto &point : visual_paths_[path_id]) {
            const double dis = std::hypot(point.x, point.y);
            if (dis > path_range / path_scale ||
                (dis > (relative_goal_distance + goal_clear_range_) /
                           path_scale &&
                 path_crop_by_goal_)) {
                continue;
            }

            *iter_x = static_cast<float>(
                path_scale * (std::cos(rot_ang) * point.x -
                              std::sin(rot_ang) * point.y));
            *iter_y = static_cast<float>(
                path_scale * (std::sin(rot_ang) * point.x +
                              std::cos(rot_ang) * point.y));
            *iter_z = static_cast<float>(path_scale * point.z);
            *iter_intensity = 1.0f;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
            ++used_points;
        }
        }
    }

    if (used_points != point_count) {
        modifier.resize(used_points);
    }

    free_paths_pub_->publish(cloud);
}

void PidPathFollower::publishEmptyFreePaths(
    const geometry_msgs::msg::PoseStamped &robot_pose)
{
    if (!shouldPublishFreePaths()) {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    if (robot_pose.header.stamp.nanosec == 0 &&
        robot_pose.header.stamp.sec == 0) {
        cloud.header.stamp = node_->now();
    } else {
        cloud.header.stamp = robot_pose.header.stamp;
    }
    cloud.header.frame_id =
        costmap_ros_ ? costmap_ros_->getBaseFrameID() : "base_link";

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(0);
    free_paths_pub_->publish(cloud);
}

void PidPathFollower::publishLocalCostmapPath(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const LocalPathSelection &selection)
{
    if (!publish_local_costmap_path_ || !local_path_pub_) {
        return;
    }
    if (local_path_pub_->get_subscription_count() == 0) {
        return;
    }

    nav_msgs::msg::Path path;
    if (robot_pose.header.stamp.nanosec == 0 &&
        robot_pose.header.stamp.sec == 0) {
        path.header.stamp = node_->now();
    } else {
        path.header.stamp = robot_pose.header.stamp;
    }
    path.header.frame_id = robot_pose.header.frame_id;
    path.poses.reserve(selection.local_path.size());

    const double robot_yaw = yawFromQuaternion(robot_pose.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    for (const auto &local_point : selection.local_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = robot_pose.pose.position.x +
                               cos_yaw * local_point.first -
                               sin_yaw * local_point.second;
        pose.pose.position.y = robot_pose.pose.position.y +
                               sin_yaw * local_point.first +
                               cos_yaw * local_point.second;
        pose.pose.position.z = robot_pose.pose.position.z;
        pose.pose.orientation = robot_pose.pose.orientation;
        path.poses.push_back(pose);
    }

    local_path_pub_->publish(path);
}

geometry_msgs::msg::PoseStamped PidPathFollower::targetPoseFromLocalPath(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const LocalPathSelection &selection) const
{
    geometry_msgs::msg::PoseStamped target = robot_pose;
    if (selection.local_path.empty()) {
        return target;
    }

    std::size_t target_index = selection.local_path.size() - 1;
    double accumulated = 0.0;
    for (std::size_t i = 1; i < selection.local_path.size(); ++i) {
        accumulated += std::hypot(
            selection.local_path[i].first - selection.local_path[i - 1].first,
            selection.local_path[i].second - selection.local_path[i - 1].second);
        if (accumulated >= lookahead_distance_) {
            target_index = i;
            break;
        }
    }

    const auto &local_target = selection.local_path[target_index];
    const double robot_yaw = yawFromQuaternion(robot_pose.pose.orientation);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    target.pose.position.x = robot_pose.pose.position.x +
                             cos_yaw * local_target.first -
                             sin_yaw * local_target.second;
    target.pose.position.y = robot_pose.pose.position.y +
                             sin_yaw * local_target.first +
                             cos_yaw * local_target.second;
    target.pose.position.z = robot_pose.pose.position.z;

    const std::size_t heading_from_index = target_index > 0 ? target_index - 1 : 0;
    const std::size_t heading_to_index =
        std::min(target_index + 1, selection.local_path.size() - 1);
    const auto &heading_from = selection.local_path[heading_from_index];
    const auto &heading_point = selection.local_path[heading_to_index];
    const double local_heading =
        std::atan2(heading_point.second - heading_from.second,
                   heading_point.first - heading_from.first);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, normalizeAngle(robot_yaw + local_heading));
    target.pose.orientation = tf2::toMsg(orientation);
    return target;
}

double PidPathFollower::distanceToSegment(double px, double py, double ax,
                                          double ay, double bx, double by,
                                          double &segment_fraction) const
{
    const double vx = bx - ax;
    const double vy = by - ay;
    const double wx = px - ax;
    const double wy = py - ay;
    const double length_sq = vx * vx + vy * vy;
    if (length_sq <= 1e-9) {
        segment_fraction = 0.0;
        return std::hypot(px - ax, py - ay);
    }

    segment_fraction = (wx * vx + wy * vy) / length_sq;
    const double t = std::clamp(segment_fraction, 0.0, 1.0);
    const double cx = ax + t * vx;
    const double cy = ay + t * vy;
    return std::hypot(px - cx, py - cy);
}

void PidPathFollower::warnCollisionThrottled(const CollisionResult &collision)
{
    const rclcpp::Time now = node_->now();
    if ((now - last_collision_warn_time_).seconds() < 1.0) {
        return;
    }

    last_collision_warn_time_ = now;
    RCLCPP_WARN(node_->get_logger(),
                "Point cloud collision on path: nearest %.2f m, points %d%s",
                collision.nearest_distance, collision.point_count,
                collision.footprint_blocked ? ", footprint blocked" : "");
}

} // namespace pid_path_follower

PLUGINLIB_EXPORT_CLASS(pid_path_follower::PidPathFollower,
                       nav2_core::Controller)
