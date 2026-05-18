#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

bool isFiniteGoal(const double x, const double y, const double yaw)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("indoor_nav_goal");

  const auto default_goal = std::numeric_limits<double>::quiet_NaN();
  const double x = node->declare_parameter<double>("x", default_goal);
  const double y = node->declare_parameter<double>("y", default_goal);
  const double yaw = node->declare_parameter<double>("yaw", default_goal);
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "map");
  const std::string action_name =
    node->declare_parameter<std::string>("action_name", "navigate_to_pose");
  const std::string behavior_tree = node->declare_parameter<std::string>("behavior_tree", "");
  const double server_timeout_sec = node->declare_parameter<double>("server_timeout_sec", 10.0);
  const double result_timeout_sec = node->declare_parameter<double>("result_timeout_sec", 0.0);

  if (!isFiniteGoal(x, y, yaw)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Goal parameters are required. Example: ros2 run nav indoor_nav_goal --ros-args "
      "-p x:=1.0 -p y:=2.0 -p yaw:=0.0");
    rclcpp::shutdown();
    return 2;
  }

  auto action_client = rclcpp_action::create_client<NavigateToPose>(node, action_name);

  RCLCPP_INFO(
    node->get_logger(), "Waiting for Nav2 action server '%s'...", action_name.c_str());
  if (!action_client->wait_for_action_server(
      std::chrono::duration<double>(server_timeout_sec)))
  {
    RCLCPP_ERROR(
      node->get_logger(), "Action server '%s' was not available after %.1f seconds.",
      action_name.c_str(), server_timeout_sec);
    rclcpp::shutdown();
    return 1;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;
  goal.pose.pose.orientation = yawToQuaternion(yaw);
  goal.behavior_tree = behavior_tree;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions goal_options;
  goal_options.feedback_callback =
    [logger = node->get_logger(), clock = node->get_clock()](
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      RCLCPP_INFO_THROTTLE(
        logger, *clock, 2000,
        "Navigation feedback: %.2f m remaining, recoveries: %d",
        feedback->distance_remaining, feedback->number_of_recoveries);
    };

  RCLCPP_INFO(
    node->get_logger(), "Sending goal in %s: x=%.3f y=%.3f yaw=%.3f",
    frame_id.c_str(), x, y, yaw);
  auto goal_handle_future = action_client->async_send_goal(goal, goal_options);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by Nav2.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Goal accepted. Waiting for result...");
  auto result_future = action_client->async_get_result(goal_handle);

  rclcpp::FutureReturnCode result_code;
  if (result_timeout_sec > 0.0) {
    result_code =
      rclcpp::spin_until_future_complete(node, result_future, std::chrono::duration<double>(
        result_timeout_sec));
  } else {
    result_code = rclcpp::spin_until_future_complete(node, result_future);
  }

  if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for navigation result.");
    action_client->async_cancel_goal(goal_handle);
    rclcpp::shutdown();
    return 1;
  }

  const auto wrapped_result = result_future.get();
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "Navigation succeeded.");
      rclcpp::shutdown();
      return 0;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Navigation was aborted.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node->get_logger(), "Navigation was canceled.");
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Navigation finished with an unknown result code.");
      break;
  }

  rclcpp::shutdown();
  return 1;
}
