#include "pointcloud_to_map/pointcloud_to_map.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_to_map::PointCloudToMap>());
  rclcpp::shutdown();

  return 0;
}
