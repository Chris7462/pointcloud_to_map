#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Eigen header
#include <Eigen/Geometry>


namespace pointcloud_to_map
{

class PointCloudToMap : public rclcpp::Node
{
public:
  PointCloudToMap();
  ~PointCloudToMap() = default;

private:
  double freq_;
  double dt_;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_buff_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;

  Eigen::Affine3d base_link_velo_link_tran_;

  void wait_for_tf();
  void run_mapping();
};

} // namespace pointcloud_to_map
