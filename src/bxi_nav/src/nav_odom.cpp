#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

bool finite_twist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

geometry_msgs::msg::Vector3 to_message(const tf2::Vector3 & input)
{
  geometry_msgs::msg::Vector3 result;
  result.x = input.x();
  result.y = input.y();
  result.z = input.z();
  return result;
}

}  // namespace

class NavOdom : public rclcpp::Node
{
public:
  NavOdom()
  : Node("nav_odom")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/aft_mapped_to_init");
    output_topic_ = declare_parameter<std::string>("output_topic", "/nav/odom");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    if (robot_base_frame_.empty()) {
      throw std::invalid_argument("nav_odom frame parameters are invalid");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::QoS(20).reliable());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::QoS(20).reliable(),
      std::bind(&NavOdom::on_odom, this, std::placeholders::_1));
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!finite_twist(msg->twist.twist)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "discarding non-finite Point-LIO twist");
      return;
    }

    geometry_msgs::msg::TransformStamped body_to_base_msg;
    try {
      body_to_base_msg = tf_buffer_->lookupTransform(
        msg->child_frame_id, robot_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for odometry frame transform: %s", ex.what());
      return;
    }

    tf2::Transform odom_to_body;
    tf2::Transform body_to_base;
    tf2::fromMsg(msg->pose.pose, odom_to_body);
    tf2::fromMsg(body_to_base_msg.transform, body_to_base);

    auto output = *msg;
    const tf2::Transform odom_to_base = odom_to_body * body_to_base;
    output.pose.pose.position.x = odom_to_base.getOrigin().x();
    output.pose.pose.position.y = odom_to_base.getOrigin().y();
    output.pose.pose.position.z = odom_to_base.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(odom_to_base.getRotation());
    output.child_frame_id = robot_base_frame_;

    const tf2::Vector3 angular_body(
      msg->twist.twist.angular.x, msg->twist.twist.angular.y,
      msg->twist.twist.angular.z);
    const tf2::Vector3 linear_body(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    const tf2::Quaternion base_to_body_rotation = body_to_base.getRotation().inverse();
    output.twist.twist.angular = to_message(tf2::quatRotate(
      base_to_body_rotation, angular_body));
    output.twist.twist.linear = to_message(tf2::quatRotate(
      base_to_body_rotation,
      linear_body + angular_body.cross(body_to_base.getOrigin())));
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string robot_base_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavOdom>());
  rclcpp::shutdown();
  return 0;
}
