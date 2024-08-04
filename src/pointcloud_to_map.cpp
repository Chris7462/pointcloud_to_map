// C++ header
#include <chrono>

// ROS header
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// local header
#include "pointcloud_to_map/pointcloud_to_map.hpp"


namespace pointcloud_to_map
{

PointCloudToMap::PointCloudToMap()
: Node("pointcloud_to_map")
{
  rclcpp::QoS qos(10);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "kitti/velo", qos,
    std::bind(&PointCloudToMap::pointcloud_callback, this, std::placeholders::_1));

  freq_ = declare_parameter("freq", 40.0);
  dt_ = 1.0 / freq_;

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "kitti/velo_to_base", qos);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_link_velo_link_tran_.setIdentity();
  wait_for_tf();

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
    std::bind(&PointCloudToMap::run_mapping, this));
}

void PointCloudToMap::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  pointcloud_buff_.push(msg);
}

void PointCloudToMap::wait_for_tf()
{
  rclcpp::Time start = rclcpp::Node::now();

  RCLCPP_INFO(
    get_logger(), "Waiting for tf transform data between frames %s and %s to become available",
    "base_link", "velo_link");

  bool transform_successful = false;

  while (!transform_successful) {
    transform_successful = tf_buffer_->canTransform(
      "base_link", "velo_link",
      tf2::TimePointZero, tf2::durationFromSec(1.0));

    if (transform_successful) {
      base_link_velo_link_tran_ = tf2::transformToEigen(
        tf_buffer_->lookupTransform("base_link", "velo_link", tf2::TimePointZero));
      RCLCPP_INFO(get_logger(), "Get the transformation from velo_link to base_link.");
      break;
    }

    rclcpp::Time now = rclcpp::Node::now();

    if ((now - start).seconds() > 20.0) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No transform between frams %s and %s available after %f seconds of waiting. This warning only prints once.",
        "base_link", "velo_link", (now - start).seconds());
    }

    if (!rclcpp::ok()) {
      return;
    }

    rclcpp::WallRate(1.0).sleep();
  }

  rclcpp::Time end = rclcpp::Node::now();
  RCLCPP_INFO(
    get_logger(), "Finished waiting for tf, waited %f seconds", (end - start).seconds());
}

void PointCloudToMap::run_mapping()
{
  RCLCPP_INFO_ONCE(get_logger(), "Running Mapping!");
  rclcpp::Time time_current = rclcpp::Node::now();

  // Publish PointCloud in base_link
  if (!pointcloud_buff_.empty()) {
    mtx_.lock();
    if ((time_current - rclcpp::Time(pointcloud_buff_.front()->header.stamp)).seconds() > 0.1) { // time sync has problem
      pointcloud_buff_.pop();
      RCLCPP_WARN(get_logger(), "Timestamp unaligned, please check your pointcloud data.");
      mtx_.unlock();
    } else {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*pointcloud_buff_.front(), *pointcloud_in);
      rclcpp::Time pointcloud_time = pointcloud_buff_.front()->header.stamp;
      pointcloud_buff_.pop();
      mtx_.unlock();

      // Transform pointcloud data from velo_link to base_link
      pcl::transformPointCloud(*pointcloud_in, *pointcloud_in, base_link_velo_link_tran_);

      sensor_msgs::msg::PointCloud2 pc_msg;
      pcl::toROSMsg(*pointcloud_in, pc_msg);
      pc_msg.header.frame_id ="base_link";
      pc_msg.header.stamp = pointcloud_time;
      pointcloud_pub_->publish(pc_msg);
    }
  }
}

}  // namespace pointcloud_to_map
