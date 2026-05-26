#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bxi_nav_node");
  // sensor_msgs::msg::PointCloud2::SharedPtr temp_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  RCLCPP_INFO(node->get_logger(), "BxiNavNode has been started.");
  // const auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
  //   "/aft_mapped_to_init", 10,
  //   [logger = node->get_logger()](const nav_msgs::msg::Odometry::SharedPtr msg) {
  //     RCLCPP_INFO(
  //       logger, "Position x: %.2f, y: %.2f, z: %.2f", msg->pose.pose.position.x,
  //       msg->pose.pose.position.y, msg->pose.pose.position.z);
  //   });

  const auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", 10,
    [](const sensor_msgs::msg::PointCloud2::SharedPtr) {
      // *temp_cloud_msg = *msg;
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
