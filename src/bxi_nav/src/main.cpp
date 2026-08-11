#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "bxi_nav_interfaces/action/follow_waypoints.hpp"
#include "bxi_nav_interfaces/action/nav_goto.hpp"
#include "bxi_nav_interfaces/msg/nav_pose.hpp"
#include "bxi_nav_interfaces/msg/nav_status.hpp"
#include "bxi_nav_interfaces/srv/set_initial_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{

// 这些 using 把较长的 ROS 2 类型名缩短，下面的网关逻辑会更容易读。
using NavGoto = bxi_nav_interfaces::action::NavGoto;
using FollowWaypoints = bxi_nav_interfaces::action::FollowWaypoints;
using NavPose = bxi_nav_interfaces::msg::NavPose;
using NavStatus = bxi_nav_interfaces::msg::NavStatus;
using SetInitialPose = bxi_nav_interfaces::srv::SetInitialPose;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;

template<typename ActionT>
using ActionServer = rclcpp_action::Server<ActionT>;

template<typename ActionT>
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

// Nav2 feedback 里的时间是 ROS Duration，这里统一转成 App 接口文档约定的秒。
double durationToSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

// App 传入的 x/y/yaw 必须都是有限数，避免 NaN/Inf 进入 Nav2。
bool isFinitePose(double x, double y, double yaw)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
}

// App 使用平面 yaw，ROS 位姿使用四元数；发目标和初始位姿前需要转换。
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

// Point-LIO/ROS 里程计给的是四元数，发布给 App 的 /nav/pose 转回 yaw。
double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion_msg)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_msg, quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

// 构造 Nav2 需要的 PoseStamped：坐标系固定用参数 frame_id，方向由 yaw 转四元数。
geometry_msgs::msg::PoseStamped makePoseStamped(
  const rclcpp::Time & stamp, const std::string & frame_id, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = yawToQuaternion(yaw);
  return pose;
}

}  // namespace

class AppNavGateway : public rclcpp::Node
{
public:
  AppNavGateway()
  : Node("app_nav_gateway")
  {
    // frame_id 是 App 目标点所在坐标系；odom_topic 触发 /nav/pose 更新并提供速度。
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // /nav/pose 和 /nav/status 使用 transient_local，让 App 新订阅后能立刻拿到最近一帧。
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pose_pub_ = this->create_publisher<NavPose>("/nav/pose", latched_qos);
    status_pub_ = this->create_publisher<NavStatus>("/nav/status", latched_qos);
    // 内部桥接：App 初始化 service 转成 Nav2 标准 /initialpose topic。
    initialpose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    // 订阅 Point-LIO 里程计，并转换成 App 面向的 /nav/pose。
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&AppNavGateway::onOdometry, this, std::placeholders::_1));

    // App 对接的 service：初始化、暂停/继续。
    init_service_ = this->create_service<SetInitialPose>(
      "/nav/init", std::bind(
        &AppNavGateway::onSetInitialPose, this, std::placeholders::_1, std::placeholders::_2));
    pause_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/nav/pause", std::bind(
        &AppNavGateway::onPause, this, std::placeholders::_1, std::placeholders::_2));
    // 网关作为 Nav2 action client，把 App action 转发到 Nav2 原生 action。
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    nav_through_poses_client_ =
      rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");

    // App 单点导航 action：/nav/goto -> /navigate_to_pose。
    goto_server_ = rclcpp_action::create_server<NavGoto>(
      this, "/nav/goto",
      std::bind(&AppNavGateway::onGotoGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AppNavGateway::onGotoCancel, this, std::placeholders::_1),
      std::bind(&AppNavGateway::onGotoAccepted, this, std::placeholders::_1));

    // App 多点巡航 action：/nav/follow_waypoints -> /navigate_through_poses。
    follow_waypoints_server_ = rclcpp_action::create_server<FollowWaypoints>(
      this, "/nav/follow_waypoints",
      std::bind(&AppNavGateway::onFollowGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AppNavGateway::onFollowCancel, this, std::placeholders::_1),
      std::bind(&AppNavGateway::onFollowAccepted, this, std::placeholders::_1));

    // 即使没有导航 feedback，也定时补发状态，方便 App 显示在线/空闲状态。
    status_timer_ = this->create_wall_timer(1s, std::bind(&AppNavGateway::publishStatus, this));
    publishStatus();
    RCLCPP_INFO(
      this->get_logger(),
      "App nav gateway ready: /nav/init /nav/goto /nav/follow_waypoints /nav/pause "
      "/nav/pose /nav/status");
  }

private:
  using GotoGoalHandle = rclcpp_action::ServerGoalHandle<NavGoto>;
  using FollowGoalHandle = rclcpp_action::ServerGoalHandle<FollowWaypoints>;

  // 收到 /nav/goto goal 时先做输入校验，再抢占导航槽位。
  // 当前实现一次只允许一个导航任务，避免两个 App 客户端互相覆盖目标。
  rclcpp_action::GoalResponse onGotoGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const NavGoto::Goal> goal)
  {
    if (!isFinitePose(goal->x, goal->y, goal->yaw)) {
      RCLCPP_WARN(this->get_logger(), "Reject /nav/goto goal with non-finite pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claimNavigationSlot() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  // App 取消 /nav/goto 时先接受取消，实际取消 Nav2 goal 在执行线程里完成。
  rclcpp_action::CancelResponse onGotoCancel(const std::shared_ptr<GotoGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // action execute 不能阻塞 executor 主线程，交给独立线程等待 Nav2 反馈和结果。
  void onGotoAccepted(const std::shared_ptr<GotoGoalHandle> goal_handle)
  {
    std::thread{std::bind(&AppNavGateway::executeGoto, this, goal_handle)}.detach();
  }

  // 多点巡航和单点导航共用同一个导航槽位，因此也要做空路径、NaN/Inf 和并发校验。
  rclcpp_action::GoalResponse onFollowGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowWaypoints::Goal> goal)
  {
    if (goal->waypoints.empty()) {
      RCLCPP_WARN(this->get_logger(), "Reject /nav/follow_waypoints goal with no waypoints");
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto & waypoint : goal->waypoints) {
      if (!isFinitePose(waypoint.x, waypoint.y, waypoint.theta)) {
        RCLCPP_WARN(this->get_logger(), "Reject /nav/follow_waypoints goal with non-finite pose");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    return claimNavigationSlot() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  // App 取消多点巡航时同样先接受，执行线程负责取消 Nav2 的底层 goal。
  rclcpp_action::CancelResponse onFollowCancel(const std::shared_ptr<FollowGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 多点巡航会等待 Nav2 action 完成，也放到独立线程里执行。
  void onFollowAccepted(const std::shared_ptr<FollowGoalHandle> goal_handle)
  {
    std::thread{std::bind(&AppNavGateway::executeFollowWaypoints, this, goal_handle)}.detach();
  }

  // 抢占导航执行权：成功后状态进入 navigating，失败说明已有任务在跑。
  bool claimNavigationSlot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (navigation_active_) {
      return false;
    }
    navigation_active_ = true;
    pause_requested_ = false;
    state_ = "navigating";
    resetMetricsLocked();
    publishStatusLocked();
    return true;
  }

  // 导航结束后释放槽位，并把最终状态发布给 App。
  void releaseNavigationSlot(const std::string & final_state)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    navigation_active_ = false;
    pause_requested_ = false;
    state_ = final_state;
    publishStatusLocked();
  }

  // /nav/goto 的主执行流程：构造 Nav2 goal -> 转发 -> 根据 Nav2 result 回填 App result。
  void executeGoto(const std::shared_ptr<GotoGoalHandle> goal_handle)
  {
    auto result = std::make_shared<NavGoto::Result>();
    const auto request = goal_handle->get_goal();

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = makePoseStamped(this->now(), frame_id_, request->x, request->y, request->yaw);

    auto started_at = this->now();
    auto wrapped = runNav2Action<NavigateToPose>(
      nav_to_pose_client_, "/navigate_to_pose", goal_handle, nav_goal,
      [this, goal_handle](const std::shared_ptr<const NavigateToPose::Feedback> & feedback) {
        // 先更新 /nav/status，再把同一份进度发布给当前 action goal。
        updateNavMetrics(
          durationToSeconds(feedback->estimated_time_remaining),
          feedback->distance_remaining,
          durationToSeconds(feedback->navigation_time),
          feedback->number_of_recoveries);

        auto app_feedback = std::make_shared<NavGoto::Feedback>();
        app_feedback->distance_remaining = feedback->distance_remaining;
        app_feedback->estimated_time_remaining =
          durationToSeconds(feedback->estimated_time_remaining);
        app_feedback->navigation_time = durationToSeconds(feedback->navigation_time);
        app_feedback->number_of_recoveries = feedback->number_of_recoveries;
        if (rclcpp::ok() && !goal_handle->is_canceling()) {
          goal_handle->publish_feedback(app_feedback);
        }
      });
    result->number_of_recoveries = numberOfRecoveries();
    result->total_time = (this->now() - started_at).seconds();

    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      result->success = true;
      result->message = "navigation succeeded";
      goal_handle->succeed(result);
      releaseNavigationSlot("succeeded");
      return;
    }
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED || goal_handle->is_canceling()) {
      result->success = false;
      result->message = "navigation canceled";
      goal_handle->canceled(result);
      releaseNavigationSlot("canceled");
      return;
    }

    result->success = false;
    result->message = "navigation failed";
    goal_handle->abort(result);
    releaseNavigationSlot("failed");
  }

  // /nav/follow_waypoints 的主执行流程：把 Pose2D 数组转成 Nav2 PoseStamped 数组。
  void executeFollowWaypoints(const std::shared_ptr<FollowGoalHandle> goal_handle)
  {
    auto result = std::make_shared<FollowWaypoints::Result>();
    const auto request = goal_handle->get_goal();

    NavigateThroughPoses::Goal nav_goal;
    nav_goal.poses.reserve(request->waypoints.size());
    for (const auto & waypoint : request->waypoints) {
      nav_goal.poses.push_back(
        makePoseStamped(this->now(), frame_id_, waypoint.x, waypoint.y, waypoint.theta));
    }

    const size_t total_waypoints = request->waypoints.size();
    auto wrapped = runNav2Action<NavigateThroughPoses>(
      nav_through_poses_client_, "/navigate_through_poses", goal_handle, nav_goal,
      [this, goal_handle, total_waypoints](
        const std::shared_ptr<const NavigateThroughPoses::Feedback> & feedback) {
        // Nav2 给剩余点数；App 接口更适合显示当前完成到第几个点。
        const uint32_t remaining = feedback->number_of_poses_remaining;
        const uint32_t completed = static_cast<uint32_t>(
          total_waypoints > remaining ? total_waypoints - remaining : 0);
        setCurrentWaypointIndex(completed);
        updateNavMetrics(
          durationToSeconds(feedback->estimated_time_remaining),
          feedback->distance_remaining,
          durationToSeconds(feedback->navigation_time),
          feedback->number_of_recoveries);

        auto app_feedback = std::make_shared<FollowWaypoints::Feedback>();
        app_feedback->current_index = completed;
        app_feedback->distance_remaining = feedback->distance_remaining;
        app_feedback->estimated_time_remaining =
          durationToSeconds(feedback->estimated_time_remaining);
        if (rclcpp::ok() && !goal_handle->is_canceling()) {
          goal_handle->publish_feedback(app_feedback);
        }
      });
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      result->success = true;
      result->message = "waypoints succeeded";
      result->completed_count = static_cast<uint32_t>(request->waypoints.size());
      result->failed_index = 0;
      goal_handle->succeed(result);
      releaseNavigationSlot("succeeded");
      return;
    }
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED || goal_handle->is_canceling()) {
      result->success = false;
      result->message = "waypoints canceled";
      result->completed_count = currentWaypointIndex();
      result->failed_index = currentWaypointIndex();
      goal_handle->canceled(result);
      releaseNavigationSlot("canceled");
      return;
    }

    result->success = false;
    result->message = "waypoints failed";
    result->completed_count = currentWaypointIndex();
    result->failed_index = currentWaypointIndex();
    goal_handle->abort(result);
    releaseNavigationSlot("failed");
  }

  // Nav2 action 转发器：等 server -> 发目标 -> 轮询结果，处理取消与暂停。
  // 暂停通过取消当前 Nav2 goal 实现；继续时会重新发送同一个 goal。
  // feedback 到 App 的映射由调用方以回调传入。
  template<typename ActionT, typename AppGoalHandleT, typename FeedbackFn>
  typename ClientGoalHandle<ActionT>::WrappedResult runNav2Action(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const char * action_name,
    const std::shared_ptr<AppGoalHandleT> app_goal_handle,
    const typename ActionT::Goal & nav_goal,
    FeedbackFn publish_feedback)
  {
    using WrappedResult = typename ClientGoalHandle<ActionT>::WrappedResult;
    WrappedResult aborted_result;
    aborted_result.code = rclcpp_action::ResultCode::ABORTED;

    while (rclcpp::ok()) {
      // Nav2 action server 没起来时直接失败，避免 App 端一直悬挂。
      if (!client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 %s action server is not available", action_name);
        return aborted_result;
      }

      auto options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
      options.feedback_callback =
        [publish_feedback](
          typename ClientGoalHandle<ActionT>::SharedPtr,
          const std::shared_ptr<const typename ActionT::Feedback> feedback) {
          publish_feedback(feedback);
        };

      auto nav_goal_handle_future = client->async_send_goal(nav_goal, options);
      if (nav_goal_handle_future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timed out sending %s goal", action_name);
        return aborted_result;
      }

      auto nav_goal_handle = nav_goal_handle_future.get();
      if (!nav_goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 rejected %s goal", action_name);
        return aborted_result;
      }

      auto result_future = client->async_get_result(nav_goal_handle);
      while (rclcpp::ok()) {
        // App action 被取消时，同步取消底层 Nav2 action。
        if (app_goal_handle->is_canceling()) {
          client->async_cancel_goal(nav_goal_handle);
          WrappedResult canceled_result;
          canceled_result.code = rclcpp_action::ResultCode::CANCELED;
          return canceled_result;
        }

        // /nav/pause 置位后取消底层目标并进入 paused；恢复后重新发送原目标。
        if (pauseRequested()) {
          client->async_cancel_goal(nav_goal_handle);
          setState("paused");
          waitUntilResumeOrCancel(app_goal_handle);
          if (app_goal_handle->is_canceling()) {
            WrappedResult canceled_result;
            canceled_result.code = rclcpp_action::ResultCode::CANCELED;
            return canceled_result;
          }
          setState("navigating");
          break;
        }

        if (result_future.wait_for(100ms) == std::future_status::ready) {
          return result_future.get();
        }
      }
    }

    return aborted_result;
  }

  // pause 状态下 action 线程停在这里，直到 App 调 /nav/pause false 或取消 goal。
  template<typename GoalHandleT>
  void waitUntilResumeOrCancel(const std::shared_ptr<GoalHandleT> goal_handle)
  {
    while (rclcpp::ok() && pauseRequested() && !goal_handle->is_canceling()) {
      std::this_thread::sleep_for(100ms);
    }
  }

  // /nav/init：App 只传 x/y/yaw，网关补齐 PoseWithCovarianceStamped 后发布给定位模块。
  void onSetInitialPose(
    const std::shared_ptr<SetInitialPose::Request> request,
    std::shared_ptr<SetInitialPose::Response> response)
  {
    if (!isFinitePose(request->x, request->y, request->yaw)) {
      response->success = false;
      response->message = "x, y and yaw must be finite";
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.stamp = this->now();
    initial_pose.header.frame_id = frame_id_;
    initial_pose.pose.pose.position.x = request->x;
    initial_pose.pose.pose.position.y = request->y;
    initial_pose.pose.pose.orientation = yawToQuaternion(request->yaw);

    // covariance 为空时使用文档默认值；传入时必须是完整 6x6 协方差矩阵。
    std::fill(initial_pose.pose.covariance.begin(), initial_pose.pose.covariance.end(), 0.0);
    if (request->covariance.size() == initial_pose.pose.covariance.size()) {
      std::copy(
        request->covariance.begin(), request->covariance.end(),
        initial_pose.pose.covariance.begin());
    } else if (request->covariance.empty()) {
      initial_pose.pose.covariance[0] = 0.25;
      initial_pose.pose.covariance[7] = 0.25;
      initial_pose.pose.covariance[35] = 0.0685;
    } else {
      response->success = false;
      response->message = "covariance must be empty or contain 36 values";
      return;
    }

    initialpose_pub_->publish(initial_pose);
    response->success = true;
    response->message = "initial pose published";
  }

  // /nav/pause：true 表示暂停，false 表示继续。
  // 这里先改网关状态；底层 action 线程会观察 pause_requested_ 并取消/重发 Nav2 goal。
  void onPause(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pause_requested_ = request->data;
      if (navigation_active_) {
        state_ = pause_requested_ ? "paused" : "navigating";
      } else if (!pause_requested_ && state_ == "paused") {
        state_ = "idle";
      }
      publishStatusLocked();
    }
    response->success = true;
    response->message = request->data ? "pause requested" : "resume requested";
  }

  // App 位姿使用 map 下的 base_link；Point-LIO odometry 只用于触发更新并保留速度。
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped robot_pose_transform;
    try {
      robot_pose_transform =
        tf_buffer_->lookupTransform(frame_id_, robot_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skip /nav/pose because TF %s -> %s is unavailable: %s",
        frame_id_.c_str(), robot_base_frame_.c_str(), ex.what());
      return;
    }

    NavPose pose;
    pose.header.stamp = robot_pose_transform.header.stamp;
    pose.header.frame_id = frame_id_;
    pose.x = robot_pose_transform.transform.translation.x;
    pose.y = robot_pose_transform.transform.translation.y;
    pose.yaw = quaternionToYaw(robot_pose_transform.transform.rotation);
    pose.twist = msg->twist.twist;
    pose_pub_->publish(pose);
  }

  // 下面几个 getter/setter 都加锁，避免 action 线程和 ROS 回调线程同时读写状态。
  bool pauseRequested()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return pause_requested_;
  }

  uint16_t numberOfRecoveries()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return number_of_recoveries_;
  }

  uint32_t currentWaypointIndex()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_waypoint_index_;
  }

  void setCurrentWaypointIndex(uint32_t current_waypoint_index)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_waypoint_index_ = current_waypoint_index;
  }

  // 只改状态字符串并立刻发布 /nav/status。
  void setState(const std::string & state)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = state;
    publishStatusLocked();
  }

  // Nav2 feedback 到达时，统一刷新 App 能看到的距离、时间和恢复次数。
  void updateNavMetrics(
    double estimated_time_remaining,
    double distance_remaining,
    double navigation_time,
    uint16_t number_of_recoveries)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    estimated_time_remaining_ = estimated_time_remaining;
    distance_remaining_ = distance_remaining;
    navigation_time_ = navigation_time;
    number_of_recoveries_ = number_of_recoveries;
    publishStatusLocked();
  }

  // 新导航任务开始前清空上一次任务残留的进度数字。
  void resetMetricsLocked()
  {
    distance_remaining_ = 0.0;
    estimated_time_remaining_ = 0.0;
    navigation_time_ = 0.0;
    number_of_recoveries_ = 0;
    current_waypoint_index_ = 0;
  }

  // timer 或回调线程调用的公开发布入口：先加锁，再交给 Locked 版本组包。
  void publishStatus()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    publishStatusLocked();
  }

  // 调用者必须已持有 state_mutex_，避免同一批状态字段被交叉修改。
  void publishStatusLocked()
  {
    NavStatus status;
    status.header.stamp = this->now();
    status.header.frame_id = frame_id_;
    status.state = state_;
    status.distance_remaining = distance_remaining_;
    status.estimated_time_remaining = estimated_time_remaining_;
    status.navigation_time = navigation_time_;
    status.number_of_recoveries = number_of_recoveries_;
    status_pub_->publish(status);
  }

  // frame_id_ 决定 App 目标点、/nav/status 和 /nav/pose 所属坐标系。
  std::string frame_id_;
  std::string robot_base_frame_;
  std::string odom_topic_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 网关内部状态由 action 执行线程、service 回调和 timer 共同访问，因此统一用锁保护。
  std::mutex state_mutex_;
  bool navigation_active_{false};
  bool pause_requested_{false};
  std::string state_{"idle"};
  double distance_remaining_{0.0};
  double estimated_time_remaining_{0.0};
  double navigation_time_{0.0};
  uint16_t number_of_recoveries_{0};
  uint32_t current_waypoint_index_{0};

  // 对 App 发布的 topic。
  rclcpp::Publisher<NavPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<NavStatus>::SharedPtr status_pub_;
  // 对内部 ROS 系统发布的桥接 topic。
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  // 订阅 Point-LIO 位姿。
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // 对 App 暴露的 service。
  rclcpp::Service<SetInitialPose>::SharedPtr init_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  // 周期发布状态，保证空闲时 App 也能看到网关在线。
  rclcpp::TimerBase::SharedPtr status_timer_;
  // 对 Nav2 的 action client。
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
  // 对 App 暴露的 action server。
  typename ActionServer<NavGoto>::SharedPtr goto_server_;
  typename ActionServer<FollowWaypoints>::SharedPtr follow_waypoints_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AppNavGateway>();
  // MultiThreadedExecutor 允许 action、service、订阅和 timer 并行处理。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
