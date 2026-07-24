// Copyright 2026 Indoor SLAM contributors
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

#ifndef NAV__INDOOR_NAV_GOAL_HPP_
#define NAV__INDOOR_NAV_GOAL_HPP_

#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

namespace indoor_slam
{
namespace nav
{

struct GoalParameters
{
  double x;
  double y;
  double yaw;
  std::string frame_id;
  std::string behavior_tree;
};

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
bool hasFinitePose(const GoalParameters & parameters);
nav2_msgs::action::NavigateToPose::Goal makeNavigationGoal(
  const GoalParameters & parameters,
  const rclcpp::Time & stamp);

}  // namespace nav
}  // namespace indoor_slam

#endif  // NAV__INDOOR_NAV_GOAL_HPP_
